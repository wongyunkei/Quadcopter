/*
 * App.cpp
 *
 *  Created on: 2015�~11��27��
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
	App::mApp->mCommunicating2->ReceivePoll();
}

void SendTask(){
	App::mApp->mCommunicating1->SendPoll();
//	App::mApp->mCommunicating2->SendPoll();
}

void printElse(){
	App::mApp->mCommunicating1->Send(0, AdditionalTools::getBuffer(0)[0]);
}

void print(){
	static int index = 0;

	switch(App::mApp->mCommunicating1->PrintType){
		case 0:
			if(index == 0){
//				App::mApp->mCommunicating1->Send(0, App::mApp->mEncoder1->getVel());
				App::mApp->mCommunicating1->Send(0, App::mApp->mControlling->Motor1PWM);
			}
			else if(index == 1){
//				App::mApp->mCommunicating1->Send(1, App::mApp->mEncoder2->getVel());
				App::mApp->mCommunicating1->Send(1, App::mApp->mControlling->Motor2PWM);
			}
			else if(index == 2){
//				App::mApp->mCommunicating1->Send(2, App::mApp->mEncoder3->getVel());
				App::mApp->mCommunicating1->Send(2, App::mApp->mControlling->Motor3PWM);
			}
			else if(index == 3){
//				App::mApp->mCommunicating1->Send(3, App::mApp->mEncoder4->getVel());
				App::mApp->mCommunicating1->Send(3, App::mApp->mControlling->Motor4PWM);
			}
			break;
		case 1:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mEncoder1->getVel());
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mEncoder2->getVel());
			}
			else if(index == 2){
				App::mApp->mCommunicating1->Send(2, App::mApp->mEncoder3->getVel());
			}
			else if(index == 3){
				App::mApp->mCommunicating1->Send(3, App::mApp->mEncoder4->getVel());
			}
			break;
		case 2:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mEncoder1->getPos());
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mEncoder2->getPos());
			}
			else if(index == 2){
				App::mApp->mCommunicating1->Send(2, App::mApp->mEncoder3->getPos());
			}
			else if(index == 3){
				App::mApp->mCommunicating1->Send(3, App::mApp->mEncoder4->getPos());
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
				App::mApp->mCommunicating1->Send(0, App::mApp->mEncoder1->getPos() * 1000);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mEncoder2->getPos() * 1000);
			}
			else if(index == 2){
				App::mApp->mCommunicating1->Send(2, App::mApp->mEncoder3->getPos() * 1000);
			}
			break;
		case 2:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mLocalization->getPos()[0] * 1000);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mLocalization->getPos()[1] * 1000);
			}
			break;
		case 3:
			if(index < 3){
				App::mApp->mCommunicating1->Send(index, App::mApp->mOmega->getOmega()[index]);
			}
			break;
		case 4:
			if(index < 1){
				App::mApp->mCommunicating1->Send(index, App::mApp->mCompass->getMag().norm());
			}
			break;
		case 5:
			if(index == 0){
				App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[0])));
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[1])));
			}
			else if(index == 2){
				App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mAcceleration->getFilteredAngle()[0])));
			}
			else if(index == 3){
				App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mAcceleration->getFilteredAngle()[1])));
			}
			break;
		case 6:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, (float)(MathTools::RadianToDegree(App::mApp->mEncoderYaw->getYaw())) * 100);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[2])) * 100);
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

void EncoderUpdate(){
	App::mApp->mEncoder1->Update(MathTools::DegreeToRadian(0));
	App::mApp->mEncoder2->Update(MathTools::DegreeToRadian(0));
	App::mApp->mEncoder3->Update(MathTools::DegreeToRadian(0));
	App::mApp->mEncoder4->Update(MathTools::DegreeToRadian(0));
}

void EncoderPrint(){

	printf("%g,%g,%g,%g\n", AdditionalTools::getBuffer(0)[0], AdditionalTools::getBuffer(1)[0], AdditionalTools::getBuffer(2)[0], (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[0])));
	printf("Q:%g,%g,%g,%g\n", App::mApp->mQuaternion->_Quaternion[0], App::mApp->mQuaternion->_Quaternion[1], App::mApp->mQuaternion->_Quaternion[2], App::mApp->mQuaternion->_Quaternion[3]);

	//	App::mApp->mUART1->Print("x: %g\ty: %g\tyaw: %g\n", App::mApp->mLocalization->ReadPos()[0] * 1000, App::mApp->mLocalization->ReadPos()[1] * 1000, Math::MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[2]));
//	App::mApp->mUART1->Print("x: %g\ty: %g\tyaw: %g\n", App::mApp->mEncoder2->ReadPos(), App::mApp->mEncoder1->ReadPos(), Math::MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[2]));
}

void LocalizationUpdate(){
	App::mApp->mLocalization->LocalizationCalc();
	App::mApp->mEncoderYaw->Update();
}

void CompassCalTask(){
	App::mApp->mHMC5883L->CalibrationPrint();
}

void CompassCalPrintResult(){
	App::mApp->mHMC5883L->CalibrationResultPrint();
}
void SpiPoll(){

	char ch[32];
//	App::mApp->mSpi1->Transfer(0,'1');
//	App::mApp->mSpi1->Transfer(0,'2');
//	App::mApp->mSpi1->Transfer(0,'3');
//	App::mApp->mSpi1->Transfer(0,'4');
//	App::mApp->mSpi1->Transfer(0,'5');
//	App::mApp->mSpi1->Transfer(0,'6');
//	App::mApp->mSpi1->Transfer(0,'7');
//	App::mApp->mSpi1->Transfer(0,'8');
//	App::mApp->mSpi1->Transfer(0,'9');
//	App::mApp->mSpi1->Transfer(0,'0');
	char data[5] = {'6','7','8','9','k'};
	App::mApp->mSpi1->setSlaveTxBuffer(data, 5);
	int length = App::mApp->mSpi1->AvailableLength;
	if(length > 0){
		App::mApp->mSpi1->AvailableLength;
		App::mApp->mSpi1->Read(ch, length);
		App::mApp->mUART1->Print("%d\t%s\n", length, ch);
	}
}

//App::App() : mTask(0), mQuaternion(0){
//	Delay::DelayMS(100);
//	mApp = this;
//	mConfig = new Config();
//	mTicks = new Ticks(true);
//	mTask = new Task();
//
//	mLed1 = new Led(mConfig->LedConf1);
//	mLed2 = new Led(mConfig->LedConf2);
//	mLed3 = new Led(mConfig->LedConf3);
//	mLed4 = new Led(mConfig->LedConf4);
////	mGPIO1 = new Led(mConfig->GPIOConf1);
//
//	mUART1 = new UART(mConfig->UART1Conf1);
////	mSpi1 = new Spi(mConfig->Spi1Conf1);
////	mSpi1 = new Spi(mConfig->Spi1Conf2);
////	mCommunicating1 = new Communicating(new Communicating::Com(Communicating::Com::__SPI, (uint32_t)mSpi1));
//	mCommunicating1 = new Communicating(new Communicating::Com(Communicating::Com::__UART, (uint32_t)mUART1));
//
////	mI2C1 = new I2C(mConfig->I2C1Conf2);
////	mMPU6050 = new MPU6050(mI2C1);
////	mHMC5883L = new HMC5883L(mMPU6050);
////	mAcceleration = new Acceleration(mMPU6050);
////	mOmega = new Omega(mMPU6050);
////	mCompass = new Compass(mHMC5883L, mAcceleration);
////	mTask->Attach(4, 0, initUpdate, false, 100, false);
////	mTask->Attach(20, 0, initCompassUpdate, false, 50, false);
////
////	Delay::DelayMS(10);
////
////	mTask->Run();
////	mCompass->Reset();
////	mEncoder1 = new Encoder(mConfig->Encoder1Conf1, 0.0239232580938729f / 1000.0f, 0);
////	mEncoder2 = new Encoder(mConfig->Encoder2Conf1, -0.0241855873642646f / 1000.0f, MathTools::PI / 2.0f);
////	mEncoder3 = new Encoder(mConfig->Encoder3Conf1, -0.0237461417341602f / 2000.0f, MathTools::PI / 2.0f);
////	mEncoderYaw = new EncoderYaw(mEncoder2, mEncoder3, 0.18f);
////	mQuaternion = new Quaternion(mAcceleration, mOmega, mEncoderYaw);
////	mQuaternion->Reset();
////	mLocalization = new Localization(mQuaternion, mEncoder2, mEncoder1, 0.0f, 0.0f);
//	printf("Started\n");
//
////	mTask->Attach(4, 0, Update, true);
////	mTask->Attach(20, 0, CompassUpdate, true);
//	mTask->Attach(2, 0, EncoderUpdate, true);
////	mTask->Attach(2, 0	, LocalizationUpdate, true);
//	mTask->Attach(20, 0, ReceiveTask, true);
//	mTask->Attach(5, 0, SendTask, true);
////	mTask->Attach(100, 50, EncoderPrint, true);
//
////	mTask->Attach(20, 7, Output, true);
//	mLed1->Blink(true, 100);
////	mTask->Attach(20, 0, CompassCalTask, false, 1500);
//	mTask->Run();
//}

App::App() : mTask(0), mQuaternion(0){
	Delay::DelayMS(10);
	mApp = this;
	mConfig = new Config();
	mTicks = new Ticks(true);
	mTask = new Task();

	mLed1 = new Led(mConfig->LedConf1);
	mGPIO1 = new Led(mConfig->GPIOConf1);
	mGPIO2 = new Led(mConfig->GPIOConf2);
	mGPIO3 = new Led(mConfig->GPIOConf3);
	mGPIO4 = new Led(mConfig->GPIOConf4);

	mUART4 = new UART(mConfig->UART4Conf1);
	mSpi2 = new Spi(mConfig->Spi2Conf1);

	mCommunicating1 = new Communicating(new Communicating::Com(Communicating::Com::__UART, (uint32_t)mUART4));
	mCommunicating2 = new Communicating(new Communicating::Com(Communicating::Com::__SPI, (uint32_t)mSpi2));

	mEncoder1 = new Encoder(mConfig->Encoder1Conf1, 0.00933f / 1000.0f, 0);
	mEncoder2 = new Encoder(mConfig->Encoder2Conf1, 0.00933f / 1000.0f, 0);
	mEncoder3 = new Encoder(mConfig->Encoder3Conf1, 0.00933f / 1000.0f, 0);
	mEncoder4 = new Encoder(mConfig->Encoder4Conf1, -0.00933f / 1000.0f, 0);
	mEncoder5 = new Encoder(mConfig->Encoder5Conf1, -0.00933f / 1000.0f, 0);
	mEncoder6 = new Encoder(mConfig->Encoder6Conf1, 0.00933f / 1000.0f, 0);

	mPWM = new PWM(mConfig->mPWMConf1);
	App::mApp->mPWM->Control1(10000);
	App::mApp->mPWM->Control2(10000);
	App::mApp->mPWM->Control3(10000);
	App::mApp->mPWM->Control4(10000);

	mControlling = new Controlling(mPWM, mEncoder3,mEncoder4,mEncoder4,mEncoder5);
	printf("Started\n");

//	mTask->Attach(4, 0, Update, true);
//	mTask->Attach(20, 0, CompassUpdate, true);
	mTask->Attach(2, 0, EncoderUpdate, true);
	mTask->Attach(2, 0, ControlTask, true);
//	mTask->Attach(2, 0	, LocalizationUpdate, true);
	mTask->Attach(20, 0, ReceiveTask, true);
	mTask->Attach(5, 0, SendTask, true);
	mTask->Attach(20, 7, print, true);
//	mTask->Attach(20, 13, printElse, true);
	mLed1->Blink(true, 100);
	mGPIO1->LedControl(true);
	mGPIO2->LedControl(true);
	mGPIO3->LedControl(true);
	mGPIO4->LedControl(true);
	mTask->Run();
}

void HardFault_Handler(){
	while(true){
		printf("HardFault\n");
		Delay::DelayMS(100);
	}
}
