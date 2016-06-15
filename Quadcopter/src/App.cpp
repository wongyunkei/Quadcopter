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
Ticks* App::mTicks = 0;
Task* App::mTask = 0;
UART* App::mUART1 = 0;
UART* App::mUART3 = 0;
UART* App::mUART4 = 0;
UART* App::mUART5 = 0;
Spi* App::mSpi1 = 0;
Spi* App::mSpi2 = 0;
Communicating* App::mCommunicating1 = 0;
Communicating* App::mCommunicating2 = 0;
Communicating* App::mCommunicating3 = 0;
Com* App::Com1 = 0;
Com* App::Com2 = 0;
Com* App::Com3 = 0;

void ControlTask(){
	App::mApp->mControlling->ControllingPoll();
}

void initUpdate(Bundle* bundle){
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
	App::mApp->mCommunicating3->ReceivePoll();
}

void SendTask(){
	App::mApp->mCommunicating1->SendPoll();
	App::mApp->mCommunicating2->SendPoll();
	App::mApp->mCommunicating3->SendPoll();
}

void SendTaskSlow(){
}

void printElse(){
	App::mApp->mCommunicating1->Send(0, AdditionalTools::getBuffer(0)[0]);
}

void print(){
	static int index = 0;
	switch(App::mApp->mCommunicating1->PrintType){
		case 0:
			if(index < 3){
				App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index])));
			}
			break;
		case 1:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mEncoder3->getVel());
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mEncoder4->getVel());
			}
			else if(index == 2){
				App::mApp->mCommunicating1->Send(2, App::mApp->mEncoder5->getVel());
			}
			else if(index == 3){
				App::mApp->mCommunicating1->Send(3, App::mApp->mEncoder6->getVel());
			}
			break;
		case 2:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mEncoder3->getPos());
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mEncoder4->getPos());
			}
			else if(index == 2){
				App::mApp->mCommunicating1->Send(2, App::mApp->mEncoder5->getPos());
			}
			else if(index == 3){
				App::mApp->mCommunicating1->Send(3, App::mApp->mEncoder6->getPos());
			}
			break;
		case 3:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mEncoder1->getPos()*1000);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mEncoder2->getPos()*1000);
			}
			break;
		case 4:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mLocalization->getPos()[0] * 1000);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mLocalization->getPos()[1] * 1000);
			}
			break;
		case 5:
			App::mApp->mCommunicating1->Send(4, AdditionalTools::getBuffer(0)[0]);
			break;
		case 6:
			if(index < 3){
				App::mApp->mCommunicating1->Send(index, (float)(App::mApp->mMPU6050->getRawOmega()[index]));
			}
			break;
		case 7:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mControlling->Motor1PWM);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mControlling->Motor2PWM);
			}
			else if(index == 2){
				App::mApp->mCommunicating1->Send(2, App::mApp->mControlling->Motor3PWM);
			}
			else if(index == 3){
				App::mApp->mCommunicating1->Send(3, App::mApp->mControlling->Motor4PWM);
			}
			break;
		case 8:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mSonic1->Distance);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mSonic2->Distance);
			}
			else if(index == 2){
				App::mApp->mCommunicating1->Send(2, App::mApp->mSonic3->Distance);
			}
			else if(index == 3){
				App::mApp->mCommunicating1->Send(3, App::mApp->mSonic4->Distance);
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
	App::mApp->mEncoder5->Update(MathTools::DegreeToRadian(0));
	App::mApp->mEncoder6->Update(MathTools::DegreeToRadian(0));
}

void LocalizationUpdate(){
	App::mApp->mLocalization->LocalizationCalc();
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

void SPITest(){
	App::mApp->mCommunicating3->Send(0,0);
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

void SonicUpdate(){
	static int index = 0;
	if(index == 0){
		App::mApp->mSonic1->Update();
		index++;
	}
	else if(index == 1){
		App::mApp->mSonic2->Update();
		index++;
	}
	else if(index == 2){
		App::mApp->mSonic3->Update();
		index++;
	}
	else if(index == 3){
		App::mApp->mSonic4->Update();
		index++;
	}
	if(index >= 4){
		index = 0;
	}

}

void PathTask(){

	float Long = 0.240f;
	float Width = 0.150f;

	PT points[20] = {{1.0,0.0, 1.0, 0, false, false, false, false},
			{1.0,1.0, 1.0, -MathTools::PI / 2, false, false, false, false},
			{0.2,1.5, 1.0, -MathTools::PI / 2, true, true, false, false, 0.3, 0.3, 0, 0, true, false, 1.2f, 0},
			{1.0,1.0, 0.3, -MathTools::PI, false, false, false, false},
			{0.2,1.0, -0.5, -MathTools::PI, true, true, false, false, 0.3, 0.3, 0, 0, false, true, 0, -0.08f},
			{1.0,0.0, 0.0, 0, 0, false, false, false, false},
			{1.0,1.0, 1.0, 0, 0, false, false, false, false},
			{1.0,1.0, 0.0, 0, 0, false, false, false, false},
			{1.0,0.0, 1.0, 0, 0, false, false, false, false},
			{1.0,0.0, 0.0, 0, 0, false, false, false, false}
			};

	int index = 9;

	if(App::mApp->PathState == 999){
		App::mApp->mControlling->MoveToTarget(1.0, 0, 0, 0);
		if(MathTools::CheckWithInInterval(App::mApp->mLocalization->getPos()[0], 0, 0.05) &&
				MathTools::CheckWithInInterval(App::mApp->mLocalization->getPos()[1], 0, 0.05) &&
				MathTools::CheckWithInInterval(App::mApp->mQuaternion->getEuler()[2], 0, 0.0174)){
			App::mApp->PathState = 0;
			App::mApp->mControlling->Stopping();
		}
	}
	else{

		if(points[App::mApp->PathState].SonicCalFL && points[App::mApp->PathState].SonicCalFR){
			if(MathTools::CheckWithInInterval(App::mApp->mSonic1->Distance - App::mApp->mSonic2->Distance, 0, 0.005f)){
				Vector3f angle = App::mApp->mQuaternion->getEuler();
				if(MathTools::CheckWithInInterval(angle[2], 0, MathTools::PI / 4)){
					angle[2] = 0;
				}
				else if(MathTools::CheckWithInInterval(angle[2], MathTools::PI / 2, MathTools::PI / 4)){
					angle[2] = MathTools::PI / 2;
				}
				else if(MathTools::CheckWithInInterval(angle[2], MathTools::PI, MathTools::PI / 4)){
					angle[2] = MathTools::PI;
				}
				else if(MathTools::CheckWithInInterval(angle[2], -MathTools::PI, MathTools::PI / 4)){
					angle[2] = -MathTools::PI;
				}
				else if(MathTools::CheckWithInInterval(angle[2], -MathTools::PI / 2, MathTools::PI / 4)){
					angle[2] = -MathTools::PI / 2;
				}
				App::mApp->mQuaternion->setEuler(angle);
			}

		if(points[App::mApp->PathState].SonicCalFL || points[App::mApp->PathState].SonicCalFR || points[App::mApp->PathState].SonicCalL || points[App::mApp->PathState].SonicCalR){
//			float extraYaw = 0;
			float extraX = 0;
			float extraY = 0;
//			float value = App::mApp->SonicPid->pid(0,App::mApp->mSonic1->Distance - App::mApp->mSonic2->Distance);//atan2f(App::mApp->mSonic1->Distance,2 * Width) - atan2f(App::mApp->mSonic2->Distance,2 * Width));
//
//			if(value != value){
//				value = 0;
//			}
//			extraYaw = (points[App::mApp->PathState].SonicCalFL && points[App::mApp->PathState].SonicCalFR) ? value : 0;
//			extraYaw *= MathTools::CheckWithInInterval(points[App::mApp->PathState].yaw, 0, MathTools::PI / 4) ? 1 :
//			MathTools::CheckWithInInterval(points[App::mApp->PathState].yaw, -MathTools::PI / 2, MathTools::PI / 4) ? -1 :
//			MathTools::CheckWithInInterval(points[App::mApp->PathState].yaw, MathTools::PI / 2, MathTools::PI / 4) ? 1 :
//			MathTools::CheckWithInInterval(points[App::mApp->PathState].yaw, MathTools::PI, MathTools::PI / 4) ? -1 : -1;
			//extraYaw = extraYaw > MathTools::PI / 24 ? MathTools::PI / 24 : extraYaw < -MathTools::PI / 24 ? -MathTools::PI / 24 : extraYaw;
//			float f[3] = {App::mApp->mSonic1->Distance, App::mApp->mSonic2->Distance, MathTools::RadianToDegree(extraYaw)};
//
//			AdditionalTools::setBuffer(0, f, 3);

//			if(extraYaw != extraYaw){
//				extraYaw = 0;
//			}
//			extraYaw = MathTools::RadianToDegree(extraYaw) < 2 && MathTools::RadianToDegree(extraYaw) > 0 ? MathTools::DegreeToRadian(2) : MathTools::RadianToDegree(extraYaw) < 0 && MathTools::RadianToDegree(extraYaw) > -2 ? -MathTools::DegreeToRadian(2) : extraYaw;
			float value1 = 0;
			float value2 = 0;
			if(points[App::mApp->PathState].SonicCalFL || points[App::mApp->PathState].SonicCalFR){
				if(points[App::mApp->PathState].SonicCalFL && points[App::mApp->PathState].SonicCalFR){
					value1 = App::mApp->mSonic1->Distance - points[App::mApp->PathState].FL + App::mApp->mSonic2->Distance - points[App::mApp->PathState].FR;
					value1 /= 2;
				}
				else if(points[App::mApp->PathState].SonicCalFL){
					value1 = App::mApp->mSonic1->Distance - points[App::mApp->PathState].FL;
				}
				else if(points[App::mApp->PathState].SonicCalFR){
					value1 = App::mApp->mSonic2->Distance - points[App::mApp->PathState].FR;
				}
			}

			if(points[App::mApp->PathState].SonicCalL || points[App::mApp->PathState].SonicCalR){
				if(points[App::mApp->PathState].SonicCalL){
					value2 = App::mApp->mSonic3->Distance - points[App::mApp->PathState].L;
				}
				else if(points[App::mApp->PathState].SonicCalFR){
					value2 = App::mApp->mSonic4->Distance - points[App::mApp->PathState].R;
				}
			}

			extraY = cosf(points[App::mApp->PathState].yaw) * value1 + sinf(points[App::mApp->PathState].yaw) * value2;
			extraX = -sinf(points[App::mApp->PathState].yaw) * value1 + cosf(points[App::mApp->PathState].yaw) * value2;

			if(points[App::mApp->PathState].SonicCalFL && points[App::mApp->PathState].SonicCalFR){
				App::mApp->mControlling->MoveToTargetWithSonicDriveYaw(points[App::mApp->PathState].speed, points[App::mApp->PathState].x + extraX, points[App::mApp->PathState].y + extraY);
			}
			else{
				App::mApp->mControlling->MoveToTarget(points[App::mApp->PathState].speed, points[App::mApp->PathState].x + extraX, points[App::mApp->PathState].y + extraY, points[App::mApp->PathState].yaw);
			}
			if(((!points[App::mApp->PathState].SonicCalFL || MathTools::CheckWithInInterval(App::mApp->mSonic1->Distance, points[App::mApp->PathState].FL, 0.01f)) &&
			   (!points[App::mApp->PathState].SonicCalFR || MathTools::CheckWithInInterval(App::mApp->mSonic2->Distance, points[App::mApp->PathState].FR, 0.01f)) &&
			   (!points[App::mApp->PathState].SonicCalL || MathTools::CheckWithInInterval(App::mApp->mSonic3->Distance, points[App::mApp->PathState].L, 0.01f)) &&
			   (!points[App::mApp->PathState].SonicCalR || MathTools::CheckWithInInterval(App::mApp->mSonic4->Distance, points[App::mApp->PathState].R, 0.01f))) ||
//			   MathTools::CheckWithInInterval(App::mApp->mQuaternion->getEuler()[2], points[App::mApp->PathState].yaw, 0.0174)) ||
//			   (!(points[App::mApp->PathState].SonicCalFL && points[App::mApp->PathState].SonicCalFR) || MathTools::CheckWithInInterval(App::mApp->mQuaternion->getEuler()[2], points[App::mApp->PathState].yaw, 0.0174))) ||
			   MathTools::CheckWithInInterval(App::mApp->mSonic1->Distance, 0.1f, 0.05f) ||
			   MathTools::CheckWithInInterval(App::mApp->mSonic2->Distance, 0.1f, 0.05f)){

					App::mApp->mControlling->Pause();
					//Delay::DelayMS(2000);

	//				extraYaw = 0;
					Vector3f value = App::mApp->mLocalization->getPos();
					if(points[App::mApp->PathState].CalX){
						value[0] = points[App::mApp->PathState].CalXValue;
						printf("X:%g\r\n", value[0]);
					}
					if(points[App::mApp->PathState].CalY){
						value[1] = points[App::mApp->PathState].CalYValue;
						printf("Y:%g\r\n", value[1]);
					}
					App::mApp->mLocalization->setPos(value);
					App::mApp->PathState++;
					App::mApp->mCommunicating1->Acknowledgement();
					if(App::mApp->PathState > index){
						App::mApp->PathState = 0;
					}
				}
			}
		}
		else{
			App::mApp->mControlling->MoveToTarget(points[App::mApp->PathState].speed, points[App::mApp->PathState].x, points[App::mApp->PathState].y, points[App::mApp->PathState].yaw);
			if(MathTools::CheckWithInInterval(App::mApp->mLocalization->getPos()[0], points[App::mApp->PathState].x, 0.05) &&
			   MathTools::CheckWithInInterval(App::mApp->mLocalization->getPos()[1], points[App::mApp->PathState].y, 0.05) &&
			   MathTools::CheckWithInInterval(App::mApp->mQuaternion->getEuler()[2], points[App::mApp->PathState].yaw, 0.0174)){
				App::mApp->PathState++;
				App::mApp->mCommunicating1->Acknowledgement();
				if(App::mApp->PathState > index){
					App::mApp->PathState = 0;
				}
			}
		}
	}
}

void PathTaskWithMyRIO(){

	float Long = 0.240f;
	float Width = 0.150f;

	if(App::mApp->currentPT.SonicCalFL && App::mApp->currentPT.SonicCalFR){
		if(MathTools::CheckWithInInterval(App::mApp->mSonic1->Distance - App::mApp->mSonic2->Distance, 0, 0.005f)){
			Vector3f angle = App::mApp->mQuaternion->getEuler();
			if(MathTools::CheckWithInInterval(angle[2], 0, MathTools::PI / 4)){
				angle[2] = 0;
			}
			else if(MathTools::CheckWithInInterval(angle[2], MathTools::PI / 2, MathTools::PI / 4)){
				angle[2] = MathTools::PI / 2;
			}
			else if(MathTools::CheckWithInInterval(angle[2], MathTools::PI, MathTools::PI / 4)){
				angle[2] = MathTools::PI;
			}
			else if(MathTools::CheckWithInInterval(angle[2], -MathTools::PI, MathTools::PI / 4)){
				angle[2] = -MathTools::PI;
			}
			else if(MathTools::CheckWithInInterval(angle[2], -MathTools::PI / 2, MathTools::PI / 4)){
				angle[2] = -MathTools::PI / 2;
			}
			App::mApp->mQuaternion->setEuler(angle);
		}

	if(App::mApp->currentPT.SonicCalFL || App::mApp->currentPT.SonicCalFR || App::mApp->currentPT.SonicCalL || App::mApp->currentPT.SonicCalR){
//			float extraYaw = 0;
		float extraX = 0;
		float extraY = 0;
//			float value = App::mApp->SonicPid->pid(0,App::mApp->mSonic1->Distance - App::mApp->mSonic2->Distance);//atan2f(App::mApp->mSonic1->Distance,2 * Width) - atan2f(App::mApp->mSonic2->Distance,2 * Width));
//
//			if(value != value){
//				value = 0;
//			}
//			extraYaw = (points[App::mApp->PathState].SonicCalFL && points[App::mApp->PathState].SonicCalFR) ? value : 0;
//			extraYaw *= MathTools::CheckWithInInterval(points[App::mApp->PathState].yaw, 0, MathTools::PI / 4) ? 1 :
//			MathTools::CheckWithInInterval(points[App::mApp->PathState].yaw, -MathTools::PI / 2, MathTools::PI / 4) ? -1 :
//			MathTools::CheckWithInInterval(points[App::mApp->PathState].yaw, MathTools::PI / 2, MathTools::PI / 4) ? 1 :
//			MathTools::CheckWithInInterval(points[App::mApp->PathState].yaw, MathTools::PI, MathTools::PI / 4) ? -1 : -1;
		//extraYaw = extraYaw > MathTools::PI / 24 ? MathTools::PI / 24 : extraYaw < -MathTools::PI / 24 ? -MathTools::PI / 24 : extraYaw;
//			float f[3] = {App::mApp->mSonic1->Distance, App::mApp->mSonic2->Distance, MathTools::RadianToDegree(extraYaw)};
//
//			AdditionalTools::setBuffer(0, f, 3);

//			if(extraYaw != extraYaw){
//				extraYaw = 0;
//			}
//			extraYaw = MathTools::RadianToDegree(extraYaw) < 2 && MathTools::RadianToDegree(extraYaw) > 0 ? MathTools::DegreeToRadian(2) : MathTools::RadianToDegree(extraYaw) < 0 && MathTools::RadianToDegree(extraYaw) > -2 ? -MathTools::DegreeToRadian(2) : extraYaw;
		float value1 = 0;
		float value2 = 0;
		if(App::mApp->currentPT.SonicCalFL || App::mApp->currentPT.SonicCalFR){
			if(App::mApp->currentPT.SonicCalFL && App::mApp->currentPT.SonicCalFR){
				value1 = App::mApp->mSonic1->Distance - App::mApp->currentPT.FL + App::mApp->mSonic2->Distance - App::mApp->currentPT.FR;
				value1 /= 2;
			}
			else if(App::mApp->currentPT.SonicCalFL){
				value1 = App::mApp->mSonic1->Distance - App::mApp->currentPT.FL;
			}
			else if(App::mApp->currentPT.SonicCalFR){
				value1 = App::mApp->mSonic2->Distance - App::mApp->currentPT.FR;
			}
		}
		else{
			App::mApp->mControlling->Pause();
		}

		if(App::mApp->currentPT.SonicCalL || App::mApp->currentPT.SonicCalR){
			if(App::mApp->currentPT.SonicCalL){
				value2 = App::mApp->mSonic3->Distance - App::mApp->currentPT.L;
			}
			else if(App::mApp->currentPT.SonicCalFR){
				value2 = App::mApp->mSonic4->Distance - App::mApp->currentPT.R;
			}
		}

		extraY = cosf(App::mApp->currentPT.yaw) * value1 + sinf(App::mApp->currentPT.yaw) * value2;
		extraX = -sinf(App::mApp->currentPT.yaw) * value1 + cosf(App::mApp->currentPT.yaw) * value2;
		if(App::mApp->trigger){
			if(App::mApp->currentPT.SonicCalFL && App::mApp->currentPT.SonicCalFR){
				App::mApp->mControlling->MoveToTargetWithSonicDriveYaw(App::mApp->currentPT.speed, App::mApp->currentPT.x + extraX, App::mApp->currentPT.y + extraY);
			}
			else{
				App::mApp->mControlling->MoveToTarget(App::mApp->currentPT.speed, App::mApp->currentPT.x + extraX, App::mApp->currentPT.y + extraY, App::mApp->currentPT.yaw);
			}
		}
		else{
			App::mApp->mControlling->Pause();
		}
		if(((!App::mApp->currentPT.SonicCalFL || MathTools::CheckWithInInterval(App::mApp->mSonic1->Distance, App::mApp->currentPT.FL, 0.01f)) &&
		   (!App::mApp->currentPT.SonicCalFR || MathTools::CheckWithInInterval(App::mApp->mSonic2->Distance, App::mApp->currentPT.FR, 0.01f)) &&
		   (!App::mApp->currentPT.SonicCalL || MathTools::CheckWithInInterval(App::mApp->mSonic3->Distance, App::mApp->currentPT.L, 0.01f)) &&
		   (!App::mApp->currentPT.SonicCalR || MathTools::CheckWithInInterval(App::mApp->mSonic4->Distance, App::mApp->currentPT.R, 0.01f))) ||
//			   MathTools::CheckWithInInterval(App::mApp->mQuaternion->getEuler()[2], points[App::mApp->PathState].yaw, 0.0174)) ||
//			   (!(points[App::mApp->PathState].SonicCalFL && points[App::mApp->PathState].SonicCalFR) || MathTools::CheckWithInInterval(App::mApp->mQuaternion->getEuler()[2], points[App::mApp->PathState].yaw, 0.0174))) ||
		   MathTools::CheckWithInInterval(App::mApp->mSonic1->Distance, 0.1f, 0.05f) ||
		   MathTools::CheckWithInInterval(App::mApp->mSonic2->Distance, 0.1f, 0.05f)){

				//Delay::DelayMS(2000);

//				extraYaw = 0;
				Vector3f value = App::mApp->mLocalization->getPos();
				if(App::mApp->currentPT.CalX){
					value[0] = App::mApp->currentPT.CalXValue;
					printf("X:%g\r\n", value[0]);
				}
				if(App::mApp->currentPT.CalY){
					value[1] = App::mApp->currentPT.CalYValue;
					printf("Y:%g\r\n", value[1]);
				}
				App::mApp->mLocalization->setPos(value);

				App::mApp->trigger = false;
				App::mApp->arrived = true;
				App::mApp->mControlling->Pause();
				App::mApp->mCommunicating1->Acknowledgement();
			}
		}
	}
	else{
		if(App::mApp->trigger){
			App::mApp->mControlling->MoveToTarget(App::mApp->currentPT.speed, App::mApp->currentPT.x, App::mApp->currentPT.y, App::mApp->currentPT.yaw);
		}
		else{
			App::mApp->mControlling->Pause();
		}
		if(MathTools::CheckWithInInterval(App::mApp->mLocalization->getPos()[0], App::mApp->currentPT.x, 0.05) &&
		   MathTools::CheckWithInInterval(App::mApp->mLocalization->getPos()[1], App::mApp->currentPT.y, 0.05) &&
		   MathTools::CheckWithInInterval(App::mApp->mQuaternion->getEuler()[2], App::mApp->currentPT.yaw, 0.0174)){

			App::mApp->trigger = false;
			App::mApp->arrived = true;
			App::mApp->mControlling->Pause();
			App::mApp->mCommunicating1->Acknowledgement();
		}
	}
}

void SPISlaveSendTask(){
	App::mApp->mCommunicating2->Send(App::mApp->PeriodicCmd, App::mApp->PeriodicData);
}

void SPISlaveSendTask2(){
	static int count = 0;
	App::mApp->mCommunicating2->Send(App::mApp->PeriodicCmd2, App::mApp->PeriodicData2);
	if(count++ > 10){
		count = 0;
		App::mApp->PeriodicCmd2 = 0;
	}
}

void SPISendTask(){
	static int count = 0;
	App::mApp->mCommunicating3->Send(App::mApp->PeriodicCmd, App::mApp->PeriodicData);
	if(App::mApp->PeriodicCmd == Communicating::CLAMPER_RESET_RUN){
		if(count++ > 10){
			count = 0;
			App::mApp->PeriodicCmd = Communicating::CMD::CLAMPER_WATCHDOG;
		}
	}
}

void printTaskNum(){
	printf("PathState:%d\r\n", App::mApp->mCommunicating3->txBufferCount);
}

void printfBufferTask(){
	AdditionalTools::printfBuffer(0, 4);
}

void Task60Hz(Bundle* bundle){
	Update();
	EncoderUpdate();
	LocalizationUpdate();
	SonicUpdate();
	ControlTask();
	ReceiveTask();
	SendTask();
}

void Task50Hz(Bundle* bundle){
//	PathTask();
	if(!App::mApp->mControlling->ManualMode){
		PathTaskWithMyRIO();
	}
	print();
}

void Task10Hz(Bundle* bundle){
	SPISlaveSendTask2();
	SPISendTask();
}

void EncoderPrint(){
	printf("M1:%g  M2:%g  M3:%g\r\n", App::mApp->mEncoder3->getPos(), App::mApp->mEncoder4->getPos(), App::mApp->mEncoder5->getPos());
	printf("T1:%g  T2:%g  T3:%g\r\n", App::mApp->Motor1Target, App::mApp->Motor2Target, App::mApp->Motor3Target);
}

void CheckPos(){
	static int index = 0;
	if(index == 0 && MathTools::CheckWithInInterval(fabs(App::mApp->Motor1Target - App::mApp->mEncoder3->getPos()), 0, 0.1f)){
		App::mApp->PeriodicCmd = Communicating::SUCCESS;
		App::mApp->PeriodicData = Communicating::CLAMPER_SET_MOTOR1_TARGET;
//		printf("SUCCESS:1\r\n");
	}
	if(index == 1 && MathTools::CheckWithInInterval(fabs(App::mApp->Motor2Target - App::mApp->mEncoder4->getPos()), 0, 0.1f)){
		App::mApp->PeriodicCmd = Communicating::SUCCESS;
		App::mApp->PeriodicData = Communicating::CLAMPER_SET_HORIZONTAL;
//		printf("SUCCESS:2\r\n");
	}
	if(index == 2 && MathTools::CheckWithInInterval(fabs(App::mApp->Motor3Target - App::mApp->mEncoder5->getPos()), 0, 0.1f)){
		App::mApp->PeriodicCmd = Communicating::SUCCESS;
		App::mApp->PeriodicData = Communicating::CLAMPER_SET_HORIZONTAL;
//		printf("SUCCESS:3\r\n");
	}
	if(index++ > 2){
		index = 0;
	}
}

void App2Controlling(){
	if(App::mApp->ControlStart){
		if(App::mApp->Motor1Target == App::mApp->Motor1Target &&
				App::mApp->Motor2Target == App::mApp->Motor2Target &&
				App::mApp->Motor3Target == App::mApp->Motor3Target &&
				App::mApp->mEncoder3->getPos() == App::mApp->mEncoder3->getPos() &&
				App::mApp->mEncoder4->getPos() == App::mApp->mEncoder4->getPos() &&
				App::mApp->mEncoder5->getPos() == App::mApp->mEncoder5->getPos()){
			float pwm1 = App::mApp->Motor1PID->pid(App::mApp->Motor1Target, App::mApp->mEncoder3->getPos());
			float pwm2 = App::mApp->Motor2PID->pid(App::mApp->Motor2Target, App::mApp->mEncoder4->getPos());
			float pwm3 = App::mApp->Motor3PID->pid(App::mApp->Motor3Target, App::mApp->mEncoder5->getPos());

			if(pwm1 == pwm1 && pwm2 == pwm2 && pwm3 == pwm3){
	//			if(MathTools::CheckWithInInterval(pwm1, 0, 20)){
	//				pwm1 = 0;
	//			}
	//			if(MathTools::CheckWithInInterval(pwm2, 0, 20)){
	//				pwm2 = 0;
	//			}
	//			if(MathTools::CheckWithInInterval(pwm3, 0, 20)){
	//				pwm3 = 0;
	//			}

				App::mApp->Motor1PWM = pwm1;
				App::mApp->Motor2PWM = pwm2;
				App::mApp->Motor3PWM = pwm3;
				App::mApp->Motor1PWM = App::mApp->Motor1PWM < -10000.0f ? -10000.0f : App::mApp->Motor1PWM  > 10000.0f ? 10000.0f : App::mApp->Motor1PWM;
				App::mApp->Motor2PWM = App::mApp->Motor2PWM < -10000.0f ? -10000.0f : App::mApp->Motor2PWM  > 10000.0f ? 10000.0f : App::mApp->Motor2PWM;
				App::mApp->Motor3PWM = App::mApp->Motor3PWM < -10000.0f ? -10000.0f : App::mApp->Motor3PWM  > 10000.0f ? 10000.0f : App::mApp->Motor3PWM;

	//			if(App::mApp->IsCal1 < 0 && MathTools::CheckWithInInterval(App::mApp->mEncoder3->getPos(), 0, 0.1) && MathTools::CheckWithInInterval(App::mApp->Motor1Target, 0, 0.01)){
	//				App::mApp->Motor1PWM = 0;
	//			}
	//			if(App::mApp->IsCal2 < 0 && MathTools::CheckWithInInterval(App::mApp->mEncoder4->getPos(), 0, 0.1) && MathTools::CheckWithInInterval(App::mApp->Motor2Target, 0, 0.01)){
	//				App::mApp->Motor2PWM = 0;
	//			}
	//			if(App::mApp->IsCal3 < 0 && MathTools::CheckWithInInterval(App::mApp->mEncoder5->getPos(), 0, 0.1) && MathTools::CheckWithInInterval(App::mApp->Motor3Target, 0, 0.01)){
	//				App::mApp->Motor3PWM = 0;
	//			}

				if(App::mApp->IsCal1 >= 0){
					if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6) == Bit_RESET){
	//					printf("Limit1\r\n");
						App::mApp->IsCal1++;
						if(App::mApp->IsCal1 > 500){
							App::mApp->IsCal1 = -100;
						}
						App::mApp->mPWM->Control1(0);
						App::mApp->mEncoder3->setPos(-2.0);
						App::mApp->Motor1Target = 0;
						App::mApp->Motor1PWM = 0;
					}
				}
				App::mApp->mPWM->Control1(fabs(App::mApp->Motor1PWM));
				if(App::mApp->IsCal2 >= 0){
					if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) == Bit_RESET){
						App::mApp->IsCal2++;
						if(App::mApp->IsCal2 > 500){
							App::mApp->IsCal2 = -100;
						}
	//					printf("Limit2\r\n");
						App::mApp->mPWM->Control2(0);
						App::mApp->mEncoder4->setPos(0);
						App::mApp->Motor2Target = 0;
						App::mApp->Motor2PWM = 0 ;
					}
				}
				App::mApp->mPWM->Control2(fabs(App::mApp->Motor2PWM));
				if(App::mApp->IsCal3 >= 0){
					if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8) == Bit_RESET){
						App::mApp->IsCal3++;
						if(App::mApp->IsCal3 > 500){
							App::mApp->IsCal3 = -100;
						}
	//					printf("Limit3\r\n");
						App::mApp->mPWM->Control3(0);
						App::mApp->mEncoder5->setPos(0);
						App::mApp->Motor3Target = 0;
						App::mApp->Motor3PWM = 0;
					}
				}
				App::mApp->mPWM->Control3(fabs(App::mApp->Motor3PWM));
			}

			if(App::mApp->Motor1PWM < 0.0f){
				App::mApp->mGPIO1->GPIOControl(false);
				App::mApp->mGPIO5->GPIOControl(true);
			}
			else{
				App::mApp->mGPIO1->GPIOControl(true);
				App::mApp->mGPIO5->GPIOControl(false);
			}
			if(App::mApp->Motor2PWM < 0.0f){
				App::mApp->mGPIO2->GPIOControl(false);
				App::mApp->mGPIO6->GPIOControl(true);
			}
			else{
				App::mApp->mGPIO2->GPIOControl(true);
				App::mApp->mGPIO6->GPIOControl(false);
			}
			if(App::mApp->Motor3PWM < 0.0f){
				App::mApp->mGPIO3->GPIOControl(false);
				App::mApp->mGPIO7->GPIOControl(true);
			}
			else{
				App::mApp->mGPIO3->GPIOControl(true);
				App::mApp->mGPIO7->GPIOControl(false);
			}
			printf("%g,%g,%g\r\n", pwm1, pwm2, pwm3);

		}
		else{
			App::mApp->mPWM->Control1(0);
			App::mApp->mPWM->Control2(0);
			App::mApp->mPWM->Control3(0);
			printf("%g,%g,%g\r\n", 0, 0, 0);
		}
	}
	//	printf("%d,%d,%d\r\n", App::mApp->IsCal1, App::mApp->IsCal2, App::mApp->IsCal3);
//	printf("%g,%g,%g\r\n", App::mApp->Motor1PWM, App::mApp->Motor2PWM, App::mApp->Motor3PWM);
}

void App2Task100Hz(Bundle* bundle){

//	App::mApp->mEncoder3->Update(MathTools::DegreeToRadian(0));
//	App::mApp->mEncoder4->Update(MathTools::DegreeToRadian(0));
//	App::mApp->mEncoder5->Update(MathTools::DegreeToRadian(0));
//	App::mApp->mEncoder6->Update(MathTools::DegreeToRadian(0));

}

void App2Task30Hz(Bundle* bundle){
	App::mApp->mEncoder3->Update(MathTools::DegreeToRadian(0));
	App::mApp->mEncoder4->Update(MathTools::DegreeToRadian(0));
	App::mApp->mEncoder5->Update(MathTools::DegreeToRadian(0));
	App::mApp->mEncoder6->Update(MathTools::DegreeToRadian(0));

//	printf("10Hz\r\n");
	App::mApp->mCommunicating1->ReceivePoll();
	App::mApp->mCommunicating2->ReceivePoll();

	App::mApp->mCommunicating1->SendPoll();
	//App::mApp->mCommunicating2->SendPoll();

	App2Controlling();
}

void App2Task10Hz(Bundle* bundle){
	//	CheckPos();
//	SPISlaveSendTask();
}

void App2Task3Hz(Bundle* bundle){
	EncoderPrint();
}

App::App() : arrived(false), PeriodicData(0), PeriodicCmd(0), PeriodicData2(0), PeriodicCmd2(0), trigger(false), Motor1Target(0), Motor2Target(0), Motor3Target(0),  mQuaternion(0), mCompass(0), mEncoderYaw(0), PathState(0){
	Delay::DelayMS(10);
	mApp = this;
	for(int i = 0; i < 16; i++){
		mExti[i] = 0;
	}

	currentPT.speed = 0;
	currentPT.x = 0;
	currentPT.y = 0;
	currentPT.yaw = 0;
	currentPT.SonicCalFL = false;
	currentPT.SonicCalFR = false;
	currentPT.SonicCalL = false;
	currentPT.SonicCalR = false;
	currentPT.FL = 0;
	currentPT.FR = 0;
	currentPT.L = 0;
	currentPT.R = 0;
	currentPT.CalX = false;
	currentPT.CalY = false;
	currentPT.CalXValue = 0;
	currentPT.CalYValue = 0;

	nextPT.speed = 0;
	nextPT.x = 0;
	nextPT.y = 0;
	nextPT.yaw = 0;
	nextPT.SonicCalFL = false;
	nextPT.SonicCalFR = false;
	nextPT.SonicCalL = false;
	nextPT.SonicCalR = false;
	nextPT.FL = 0;
	nextPT.FR = 0;
	nextPT.L = 0;
	nextPT.R = 0;
	nextPT.CalX = false;
	nextPT.CalY = false;
	nextPT.CalXValue = 0;
	nextPT.CalYValue = 0;

	mConfig = new Config();
	mTicks = new Ticks(false);
	mTask = new Task();

	mLed1 = new GPIO(mConfig->LedConf1);

	mGPIO1 = new GPIO(mConfig->GPIOConf1);
	mGPIO2 = new GPIO(mConfig->GPIOConf2);
	mGPIO3 = new GPIO(mConfig->GPIOConf3);
	mGPIO4 = new GPIO(mConfig->GPIOConf4);
	mGPIO5 = new GPIO(mConfig->GPIOConf5);
	mGPIO6 = new GPIO(mConfig->GPIOConf6);
	mGPIO7 = new GPIO(mConfig->GPIOConf7);
	mGPIO8 = new GPIO(mConfig->GPIOConf8);

	mUART4 = new UART(mConfig->UART4Conf1);
	mSpi1 = new Spi(mConfig->Spi1Conf1);
	mSpi2 = new Spi(mConfig->Spi2Conf1);

	Com1 = new Com(Com::__UART, (uint32_t)mUART4);
	Com2 = new Com(Com::__SPI, (uint32_t)mSpi2, 0);
	Com3 = new Com(Com::__SPI, (uint32_t)mSpi1, 0);
	mCommunicating1 = new Communicating(Com1);
	mCommunicating2 = new Communicating(Com2);
	mCommunicating3 = new Communicating(Com3);
//	mCommunicating1 = new Communicating(new Communicating::Com(Communicating::Com::__UART, (uint32_t)mUART4));
//	mCommunicating2 = new Communicating(new Communicating::Com(Communicating::Com::__SPI, (uint32_t)mSpi2));
//	mCommunicating3 = new Communicating(new Communicating::Com(Communicating::Com::__SPI, (uint32_t)mSpi1));

	mSonic1 = new Sonic(mConfig->SonicConf1);
	mSonic2 = new Sonic(mConfig->SonicConf2);
	mSonic3 = new Sonic(mConfig->SonicConf3);
	mSonic4 = new Sonic(mConfig->SonicConf4);

	mEncoder1 = new Encoder(mConfig->Encoder1Conf1, -0.008335f / 1000.0f, 0);
	mEncoder2 = new Encoder(mConfig->Encoder2Conf1, -0.008335f / 1000.0f, 0);
	mEncoder3 = new Encoder(mConfig->Encoder3Conf1, 0.00933f / 1000.0f, 0);
	mEncoder4 = new Encoder(mConfig->Encoder4Conf1, -0.00933f / 1000.0f, 0);
	mEncoder5 = new Encoder(mConfig->Encoder5Conf1, -0.00933f / 1000.0f, 0);
	mEncoder6 = new Encoder(mConfig->Encoder6Conf1, 0.00933f / 1000.0f, 0);
//	mEncoder3 = new Encoder(mConfig->Encoder3Conf1, 0.00933f / 1000.0f, 0);
//	mEncoder4 = new Encoder(mConfig->Encoder4Conf1, 0.00933f / 1000.0f, 0);
//	mEncoder5 = new Encoder(mConfig->Encoder5Conf1, 0.00933f / 1000.0f, 0);

	mPWM = new PWM(mConfig->mPWMConf1);
	App::mApp->mPWM->Control1(0);
	App::mApp->mPWM->Control2(0);
	App::mApp->mPWM->Control3(0);
	App::mApp->mPWM->Control4(0);

	mControlling = new Controlling(mPWM, mEncoder3,mEncoder4,mEncoder5,mEncoder6);

	mI2C1 = new I2C(mConfig->I2C1Conf2);
	mMPU6050 = new MPU6050(mI2C1);
	mAcceleration = new Acceleration(mMPU6050);
	mOmega = new Omega(mMPU6050);
	mTask->Attach(4, initUpdate, "initUpdate", false, 100, false);

	Delay::DelayMS(10);

	mTask->Run();
//	mEncoderYaw = new EncoderYaw(mEncoder2, mEncoder3, 0.18f);
	mQuaternion = new Quaternion(mAcceleration, mOmega);
	mQuaternion->Reset();
	mLocalization = new Localization(mQuaternion, mEncoder2, mEncoder1, -0.001f, 0.0f);



	mTask->Attach(16, Task60Hz, "Task60Hz", true);
	mTask->Attach(20, Task50Hz, "Task50Hz", true);
	mTask->Attach(100, Task10Hz, "Task10Hz", true);
//	mTask->Attach(4, 0, Update, true);
//	mTask->Attach(4, 0, EncoderUpdate, true);
//	mTask->Attach(4, 0, LocalizationUpdate, true);
//	mTask->Attach(20, 0, SonicUpdate, true);
//	mTask->Attach(20, 0, ControlTask, true);
//	mTask->Attach(4, 0, PathTask, true);
//	mTask->Attach(5, 3, ReceiveTask, true);
//	mTask->Attach(1, 0, SendTask, true);
//	mTask->Attach(1, 0, SendTaskSlow, true);
//	mTask->Attach(20, 7, print, true);
//	mTask->Attach(100, 0, SPISendTask, true);
//	mTask->Attach(1000, 0, printTaskName, "printTaskName", true);
//	mTask->Attach(100, 7, printfBufferTask, true);
	mGPIO1->GPIOControl(true);
	mGPIO2->GPIOControl(true);
	mGPIO3->GPIOControl(true);
	mGPIO4->GPIOControl(true);
	mGPIO5->GPIOControl(false);
	mGPIO6->GPIOControl(false);
	mGPIO7->GPIOControl(false);
	mGPIO8->GPIOControl(false);
	mLed1->Blink(mLed1, true, 100);
	printf("Started\n");
	mTask->Run();
}
/*
App::App() : Motor1PWM(0), Motor2PWM(0), Motor3PWM(0), arrived(false), PeriodicData(0), PeriodicCmd(0), PeriodicData2(0), PeriodicCmd2(0), trigger(false), Motor1Target(0), Motor2Target(0), Motor3Target(0), ControlStart(false), IsCal1(-100), IsCal2(-100), IsCal3(-100), mQuaternion(0), mCompass(0), mEncoderYaw(0), PathState(0){
	Delay::DelayMS(10);
	mApp = this;
	for(int i = 0; i < 16; i++){
		mExti[i] = 0;
	}

	mConfig = new Config();
	mTicks = new Ticks(false);
	mTask = new Task();

	mLed1 = new GPIO(mConfig->LedConf1);

	mGPIO1 = new GPIO(mConfig->GPIOConf1);
	mGPIO2 = new GPIO(mConfig->GPIOConf2);
	mGPIO3 = new GPIO(mConfig->GPIOConf3);
	mGPIO4 = new GPIO(mConfig->GPIOConf4);
	mGPIO5 = new GPIO(mConfig->GPIOConf5);
	mGPIO6 = new GPIO(mConfig->GPIOConf6);
	mGPIO7 = new GPIO(mConfig->GPIOConf7);
	mGPIO8 = new GPIO(mConfig->GPIOConf8);

	mUART4 = new UART(mConfig->UART4Conf1);
	mSpi2 = new Spi(mConfig->Spi2Conf1);

	Com1 = new Com(Com::__UART, (uint32_t)mUART4);
	Com2 = new Com(Com::__SPI, (uint32_t)mSpi2, 0);
	mCommunicating1 = new Communicating(Com1);
	mCommunicating2 = new Communicating(Com2);
//	mCommunicating1 = new Communicating(new Communicating::Com(Communicating::Com::__UART, (uint32_t)mUART4));
//	mCommunicating2 = new Communicating(new Communicating::Com(Communicating::Com::__SPI, (uint32_t)mSpi2));

	mEncoder3 = new Encoder(mConfig->Encoder3Conf1, 0.00933f / 1000.0f, 0);
	//car1
//	mEncoder4 = new Encoder(mConfig->Encoder4Conf1, -0.00933f / 1000.0f, 0);
//	mEncoder5 = new Encoder(mConfig->Encoder5Conf1, -0.00933f / 1000.0f, 0);
	//car2
	mEncoder4 = new Encoder(mConfig->Encoder4Conf1, 0.00933f / 1000.0f, 0);
	mEncoder5 = new Encoder(mConfig->Encoder5Conf1, 0.00933f / 1000.0f, 0);

	mEncoder6 = new Encoder(mConfig->Encoder6Conf1, 0.00933f / 1000.0f, 0);

	mPWM = new PWM(mConfig->mPWMConf1);
	App::mApp->mPWM->Control1(0);
	App::mApp->mPWM->Control2(0);
	App::mApp->mPWM->Control3(0);
	App::mApp->mPWM->Control4(0);
	mGPIO1->GPIOControl(true);
	mGPIO2->GPIOControl(true);
	mGPIO3->GPIOControl(true);
	mGPIO4->GPIOControl(true);
	mGPIO5->GPIOControl(false);
	mGPIO6->GPIOControl(false);
	mGPIO7->GPIOControl(false);
	mGPIO8->GPIOControl(false);
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

//	mExti[0] = new ExternalInterrupt(new Configuration(RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_6), ExternalInterrupt::RISING, Motor1Limit);
//	mExti[1] = new ExternalInterrupt(new Configuration(RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_7), ExternalInterrupt::RISING, Motor2Limit);
//	mExti[2] = new ExternalInterrupt(new Configuration(RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_8), ExternalInterrupt::RISING, Motor3Limit);

	Motor1PID = new Pid(20000, 0, 0.0, 10000);
	Motor2PID = new Pid(20000, 0, 0.0, 10000);
	Motor3PID = new Pid(20000, 0, 0.0, 10000);

//	mTask->Attach(10, 0, App2Task100Hz, "App2Task100Hz", true);
	mTask->Attach(20, App2Task30Hz, "App2Task30Hz", true);
	mTask->Attach(100, App2Task10Hz, "App2Task10Hz", true);
	mTask->Attach(300, App2Task3Hz, "App2Task3Hz", true);
//	mTask->Attach(4, 0, Update, true);
//	mTask->Attach(4, 0, EncoderUpdate, true);
//	mTask->Attach(4, 0, LocalizationUpdate, true);
//	mTask->Attach(20, 0, SonicUpdate, true);
//	mTask->Attach(20, 0, ControlTask, true);
//	mTask->Attach(4, 0, PathTask, true);
//	mTask->Attach(5, 3, ReceiveTask, true);
//	mTask->Attach(1, 0, SendTask, true);
//	mTask->Attach(1, 0, SendTaskSlow, true);
//	mTask->Attach(20, 7, print, true);
//	mTask->Attach(100, 0, SPISendTask, true);
//	mTask->Attach(1000, 0, printTaskName, "printTaskName", true);
//	mTask->Attach(100, 7, printfBufferTask, true);

//	mLed1->Blink(true, 100);
	printf("Started\n");
	mTask->Run();
}*/

void HardFault_Handler(){
	printf("HF:%s:%d\r\n", App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->TaskName.c_str(), App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->duration[1] - App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->duration[0]);
	App::mTicks->PrintTime();
	while(true);
}
