/*
 * App.cpp
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#include <App.h>
#include <stdio.h>
#include <stdlib.h>
using namespace std;
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

void print(){
	printf("%g,%g,%g\n", App::mApp->mOmega->getOmega()[0]
					  , App::mApp->mOmega->getOmega()[1]
					  , App::mApp->mOmega->getOmega()[2]);
}

void Output(){
	static int index = 0;
	switch(App::mApp->mCommunicating1->PrintType){
		case 0:
			if(index < 3){
				App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index])));
			}
			break;
		case 1:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mEncoder1->ReadPos()*1000);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mEncoder2->ReadPos()*1000);
			}
			break;
		case 2:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mLocalization->ReadPos()[0]*1000);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mLocalization->ReadPos()[1]*1000);
			}
			break;
		case 3:
			if(index < 3){
				App::mApp->mCommunicating1->Send(index, App::mApp->mOmega->getOmega()[index]);
			}
			break;
		case 4:
			if(index < 3){
				App::mApp->mCommunicating1->Send(index, App::mApp->mCompass->getMag().norm());
			}
			break;

	}
	if(index == 4){
		index = 0;
	}
	else{
		index++;
	}
}

void BatteryPrint(){
	App::mApp->mADCFilter->Update(App::mApp->mADC->getReading() * 3.3 / 4096.0);
	App::mApp->mCommunicating1->Send(0, App::mApp->mADCFilter->getAverage());
//	printf("%g\n", App::mApp->mADC->getReading());
}

void EncoderPoll(){
	App::mApp->mEncoder1->Poll();
	App::mApp->mEncoder2->Poll();
}

void EncoderPrint(){
	App::mApp->mUART1->Print("x: %g\ty: %g\tyaw: %g\n", App::mApp->mLocalization->ReadPos()[0] * 1000, App::mApp->mLocalization->ReadPos()[1] * 1000, Math::MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[2]));
//	App::mApp->mUART1->Print("x: %g\ty: %g\tyaw: %g\n", App::mApp->mEncoder2->ReadPos(), App::mApp->mEncoder1->ReadPos(), Math::MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[2]));
}

void LocalizationCalc(){
	App::mApp->mLocalization->LocalizationCalc();
}

void CompassCalTask(){
	App::mApp->mHMC5883L->CalibrationPrint();
}

void CompassCalPrintResult(){
	App::mApp->mHMC5883L->CalibrationResultPrint();
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
	mCommunicating1 = new Communicating(new Communicating::Com(Communicating::Com::__UART, (uint32_t)mUART1));
	mI2C1 = new I2C(mConfig->I2C1Con2);
	mMPU6050 = new MPU6050(mI2C1);
	mHMC5883L = new HMC5883L(mMPU6050);
	mAcceleration = new Acceleration(mMPU6050);
	mOmega = new Omega(mMPU6050);
	mCompass = new Compass(mHMC5883L, mAcceleration);
	mTask->Attach(4, 0, initUpdate, false, 100, false);
	mTask->Attach(20, 0, initCompassUpdate, false, 50, false);
	mTask->Run();
	mCompass->Reset();
	mQuaternion = new Quaternion(mAcceleration, mOmega, 0.004f, true, mCompass);
	mQuaternion->Reset();
	mTask->Attach(4, 0, Update, true);
	mTask->Attach(20, 0, CompassUpdate, true);
	mTask->Attach(20, 0, ReceiveTask, true);
	mTask->Attach(5, 0, SendTask, true);

	mEncoder1 = new Encoder(mConfig->Encoder1Conf1, 0.0131, 0.004f);
	mEncoder2 = new Encoder(mConfig->Encoder2Conf1, -0.0122, 0.004f);
	mLocalization = new Localization(mQuaternion, mEncoder2, mEncoder1, -0.02f, 0.075f, 0.004f);
	printf("Started\n");
	mTask->Attach(4, 0, EncoderPoll, true);
	mTask->Attach(4, 0, LocalizationCalc, true);
//	mTask->Attach(100, 50, EncoderPrint, true);

	mTask->Attach(20, 0, Output, true);
	mLed1->Blink(true, 100);
//	mTask->Attach(20, 0, CompassCalTask, false, 1500);
	mTask->Run();
}

/*
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
//	mADC = new ADConverter(mConfig->ADCConf1);
	mPWM = new PWM(mConfig->mPWMConf1);
	mControlling = new Controlling(mPWM);
	mI2C1 = new I2C(mConfig->I2C1Con2);
	mMPU6050 = new MPU6050(mI2C1);
	mHMC5883L = new HMC5883L(mMPU6050);
	mAcceleration = new Acceleration(mMPU6050);
	mOmega = new Omega(mMPU6050);
	mCompass = new Compass(mHMC5883L, mAcceleration);
	mTask->Attach(2, 0, initUpdate, false, 100, false);
	mTask->Attach(20, 0, initCompassUpdate, false, 50, false);
	mTask->Run();
	mCompass->Reset();
	mQuaternion = new Quaternion(mAcceleration, mOmega, mCompass, 0.002f);
	mQuaternion->Reset();
	mTask->Attach(2, 0, Update, true);
//	mTask->Attach(2, 1, ControlTask, true);
	mTask->Attach(20, 0, CompassUpdate, true);
	mTask->Attach(20, 0, ReceiveTask, true);
	mTask->Attach(5, 3, SendTask, true);
//	mLed1->Blink(true, 100, 4);
//	mLed2->Blink(true, 100, 4);
//	mLed3->Blink(true, 100, 4);
//	mLed4->Blink(true, 100, 4);
//	mTask->Attach(20, 0, print, true);
	mTask->Attach(10, 0, Output, true);
////	mTask->Attach(100, 0, BatteryPrint, true);
//	mLed1->LedControl(true);
//	mLed2->LedControl(true);
//	mLed3->LedControl(true);
//	mLed4->LedControl(true);

	mLed1->Blink(true, 100);
	mLed2->Blink(true, 100);
	mLed3->Blink(true, 100);
	mLed4->Blink(true, 100);
	mTask->Run();
}
*/
void HardFault_Handler(){
	while(true){
		printf("HardFault\n");
		Delay::DelayMS(100);
	}
}
