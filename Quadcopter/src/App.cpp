/*
 * App.cpp
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#include <App.h>

using namespace System;
using namespace Sensors;
using namespace Utility;

App* App::mApp = 0;

void ControlTask(){
	App::mApp->mControlling->ControllingPoll();
}

void initUpdate(){
	App::mApp->mMPU6050->Update();
	App::mApp->mAcceleration->Update();
	App::mApp->mOmega->Update();
}

void initCompassUpdate(){
	App::mApp->mHMC5883L->Update();
	App::mApp->mCompass->Update();
}

void Update(){
	App::mApp->mMPU6050->Update();
	App::mApp->mHMC5883L->Update();
	App::mApp->mAcceleration->Update();
	App::mApp->mOmega->Update();
	App::mApp->mQuaternion->Update();
}

void CompassUpdate(){
	App::mApp->mHMC5883L->Update();
	App::mApp->mCompass->Update();
}

void ReceiveTask(){
	App::mApp->mCommunicating1->ReceivePoll();
}

void SendTask(){
	App::mApp->mCommunicating1->SendPoll();
}

void Output(){
	static int index = 0;
	float offset[3] = {App::mApp->mControlling->RollOffset, App::mApp->mControlling->PitchOffset, App::mApp->mControlling->YawOffset};
	switch(App::mApp->mCommunicating1->PrintType){
		case 0:
			if(index < 3){
				App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index])) - offset[index]);
			}
			break;
		case 1:
			App::mApp->mCommunicating1->Send(index, AdditionalTools::getBuffer(index)[0]);
			break;
	}
	if(index == 3){
		index = 0;
	}
	else{
		index++;
	}
}

void BatteryPrint(){
	printf("%g\n", 4096 * 0.52 / App::mApp->mADC->getReading());
}

App::App(){
	Delay::DelayMS(100);
	mApp = this;
	mConfig = new Config();
	mTicks = new Ticks(true);
	mTask = new Task();
	mLed1 = new Led(mConfig->LedConf1);
	mLed2 = new Led(mConfig->LedConf2);
	mLed3 = new Led(mConfig->LedConf3);
	mLed4 = new Led(mConfig->LedConf4);
	mUART1 = new UART(mConfig->UART1Conf1);
	printf("Started\n");
	mCommunicating1 = new Communicating(new Communicating::Com(Communicating::Com::__UART, (uint32_t)mUART1));
	mADC = new ADConverter(mConfig->ADCConf1);
	mPWM = new PWM(mConfig->mPWMConf1);
	mControlling = new Controlling(mPWM);
	mI2C2 = new I2C(mConfig->I2C2Con1);
	mMPU6050 = new MPU6050(mI2C2);
	mHMC5883L = new HMC5883L(mMPU6050);
	mAcceleration = new Acceleration(mMPU6050);
	mOmega = new Omega(mMPU6050);
	mCompass = new Compass(mHMC5883L, mAcceleration);
	mTask->Attach(2, 0, initUpdate, false, 500, false);
	mTask->Attach(20, 0, initCompassUpdate, false, 50, false);
	mTask->Run();
	mCompass->Reset();
	mQuaternion = new Quaternion(mAcceleration, mOmega, mCompass, 0.002f);
	mQuaternion->Reset();
	mTask->Attach(2, 0, Update, true);
	mTask->Attach(2, 1, ControlTask, true);
	mTask->Attach(20, 0, CompassUpdate, true);
	mTask->Attach(20, 3, ReceiveTask, true);
	mTask->Attach(20, 7, SendTask, true);
	mLed1->Blink(true, 100, 4);
	mLed2->Blink(true, 100, 4);
	mLed3->Blink(true, 100, 4);
	mLed4->Blink(true, 100, 4);
	mTask->Attach(50, 0, Output, true);
	mTask->Attach(1000, 0, BatteryPrint, true);
	mLed1->LedControl(true);
	mLed2->LedControl(true);
	mLed3->LedControl(true);
	mLed4->LedControl(true);
	mTask->Run();
}

void HardFault_Handler(){
	while(true){
		printf("HardFault\n");
		Delay::DelayMS(100);
	}
}
