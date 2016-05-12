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

void EncoderPrint(){

	printf("%g,%g,%g,%g\n", AdditionalTools::getBuffer(0)[0], AdditionalTools::getBuffer(1)[0], AdditionalTools::getBuffer(2)[0], (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[0])));
	printf("Q:%g,%g,%g,%g\n", App::mApp->mQuaternion->_Quaternion[0], App::mApp->mQuaternion->_Quaternion[1], App::mApp->mQuaternion->_Quaternion[2], App::mApp->mQuaternion->_Quaternion[3]);

	//	App::mApp->mUART1->Print("x: %g\ty: %g\tyaw: %g\n", App::mApp->mLocalization->ReadPos()[0] * 1000, App::mApp->mLocalization->ReadPos()[1] * 1000, Math::MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[2]));
//	App::mApp->mUART1->Print("x: %g\ty: %g\tyaw: %g\n", App::mApp->mEncoder2->ReadPos(), App::mApp->mEncoder1->ReadPos(), Math::MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[2]));
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
	App::mApp->mSonic1->Update();
	App::mApp->mSonic2->Update();
	App::mApp->mSonic3->Update();
	App::mApp->mSonic4->Update();
}

void PathTask(){

	float Long = 0.240f;
	float Width = 0.150f;

	typedef struct Point{
		float speed;
		float x;
		float y;
		float yaw;
		bool SonicCalFL;
		bool SonicCalFR;
		bool SonicCalL;
		bool SonicCalR;
		float FL;
		float FR;
		float L;
		float R;
		bool CalX;
		bool CalY;
		float CalXValue;
		float CalYValue;
	} PT;

	PT points[20] = {{1.0,0.0, 1.0, 0, false, false, false, false},
			{1.0,1.0, 1.0, -MathTools::PI / 2, false, false, false, false},
			{0.1,1.5, 1.0, -MathTools::PI / 2, true, true, false, false, 0.3, 0.3, 0, 0, true, false, 1.2f, 0},
			{1.0,1.0, 0.1, -MathTools::PI, false, false, false, false},
			{0.1,1.0, -0.5, -MathTools::PI, true, true, false, false, 0.3, 0.3, 0, 0, false, true, 0, -0.08f},
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

					App::mApp->mControlling->StopAllMotors();
					Delay::DelayMS(2000);

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

void SPISendTask(){
	static int count = 0;
	App::mApp->mCommunicating3->Send(App::mApp->PeriodicCmd, App::mApp->PeriodicData);
	if(App::mApp->PeriodicCmd == 1 && count++ > 10){
		count = 0;
		App::mApp->PeriodicCmd = 0;
	}
}

void printTaskNum(){
	printf("PathState:%d\r\n", App::mApp->mCommunicating3->txBufferCount);
}

void printTaskName(){
	printf("Update:%lx\r\n", Update);
	printf("EncoderUpdate:%lx\r\n", EncoderUpdate);
	printf("LocalizationUpdate:%lx\r\n", LocalizationUpdate);
	printf("SonicUpdate:%lx\r\n", SonicUpdate);
	printf("ControlTask:%lx\r\n", ControlTask);
	printf("PathTask:%lx\r\n", PathTask);
	printf("ReceiveTask:%lx\r\n", ReceiveTask);
	printf("SendTask:%lx\r\n", SendTask);
	printf("SendTaskSlow:%lx\r\n", SendTaskSlow);
	printf("print:%lx\r\n", print);
	printf("SPISendTask:%lx\r\n", SPISendTask);
	for(int i = 0; i < App::mApp->mTask->TasksNum; i++){
		App::mApp->mTask->printDeration(i);
	}
}

void printfBufferTask(){
	AdditionalTools::printfBuffer(0, 4);
}

void Task250Hz(){
	Update();
	EncoderUpdate();
	LocalizationUpdate();
	PathTask();
}

void Task200Hz(){
	ReceiveTask();
	SendTask();
}

void Task50Hz(){
	SonicUpdate();
	ControlTask();
	SPISendTask();
	print();
}

App::App() : PeriodicCmd(0), PeriodicData(0), mTask(0), mQuaternion(0), mCompass(0), mEncoderYaw(0), PathState(0){
	Delay::DelayMS(10);
	mApp = this;
	for(int i = 0; i < 16; i++){
		mExti[i] = 0;
	}
	mConfig = new Config();
	mTicks = new Ticks(false);
	mTask = new Task();

	mLed1 = new Led(mConfig->LedConf1);

	mGPIO1 = new Led(mConfig->GPIOConf1);
	mGPIO2 = new Led(mConfig->GPIOConf2);
	mGPIO3 = new Led(mConfig->GPIOConf3);
	mGPIO4 = new Led(mConfig->GPIOConf4);
	mGPIO5 = new Led(mConfig->GPIOConf5);
	mGPIO6 = new Led(mConfig->GPIOConf6);
	mGPIO7 = new Led(mConfig->GPIOConf7);
	mGPIO8 = new Led(mConfig->GPIOConf8);

	mUART4 = new UART(mConfig->UART4Conf1);
	mSpi1 = new Spi(mConfig->Spi1Conf1);
	mSpi2 = new Spi(mConfig->Spi2Conf1);

	mCommunicating1 = new Communicating(new Communicating::Com(Communicating::Com::__UART, (uint32_t)mUART4));
	mCommunicating2 = new Communicating(new Communicating::Com(Communicating::Com::__SPI, (uint32_t)mSpi2));
	mCommunicating3 = new Communicating(new Communicating::Com(Communicating::Com::__SPI, (uint32_t)mSpi1));

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
	mTask->Attach(4, 0, initUpdate, "initUpdate", false, 100, false);

	Delay::DelayMS(10);

	mTask->Run();
//	mEncoderYaw = new EncoderYaw(mEncoder2, mEncoder3, 0.18f);
	mQuaternion = new Quaternion(mAcceleration, mOmega);
	mQuaternion->Reset();
	mLocalization = new Localization(mQuaternion, mEncoder2, mEncoder1, -0.001f, 0.0f);



	mTask->Attach(6, 0, Task250Hz, "Task250Hz", true);
	mTask->Attach(10, 0, Task200Hz, "Task250Hz", true);
	mTask->Attach(20, 0, Task50Hz, "Task50Hz", true);
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
	mGPIO1->LedControl(true);
	mGPIO2->LedControl(true);
	mGPIO3->LedControl(true);
	mGPIO4->LedControl(true);
	mGPIO5->LedControl(false);
	mGPIO6->LedControl(false);
	mGPIO7->LedControl(false);
	mGPIO8->LedControl(false);
//	mLed1->Blink(true, 100);
	printf("Started\n");
	mTask->Run(true);
}

void HardFault_Handler(){
	while(true){
		printf("HardFault:%d\n", App::mApp->mTask->currentTaskNum);
		Delay::DelayMS(100);
	}
}
