/*
 * Main.cpp
 *
 *  Created on: 2014¦~8¤ë2¤é
 *      Author: YunKei
 */

#include <inttypes.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Ticks.h>
#include <Task.h>
#include <Leds.h>
#include <Delay.h>
#include <Usart.h>
#include <I2C.h>
#include <MPU6050.h>
#include <Controlling.h>
#include <Acceleration.h>
#include <Omega.h>
#include <Quaternion.h>
#include <MathTools.h>
#include <AdditionalTools.h>
//#include <Vector.h>
#include <Communicating.h>
//#include <NRF905.h>
//#include <stm32f4xx_dma.h>
#include <Battery.h>
#include <Sonic.h>
//#include <PWM.h>
//#include <PhasesMonitoring.h>
//#include <stm32f4xx_it.h>
//#include <Buzzer.h>
//#include <PX4FLOW.h>
//#include <SE3.h>
//#include <Kalman.h>
//#include <HMC5883L.h>

void Sampling(){
	Usart::getInstance(USART1)->Print("%g,%g,%g\n", MathTools::RadianToDegree(Quaternion::getInstance(0)->getEuler(0)),
			MathTools::RadianToDegree(Quaternion::getInstance(0)->getEuler(1)),
			MathTools::RadianToDegree(Quaternion::getInstance(0)->getEuler(2)));
}

void printParams(){
//	Communicating::getInstant(Communicating::COM1)->Send(4, AdditionalTools::getBuffer(0)[0]);
	Communicating::getInstant(Communicating::COM1)->Send(4, AdditionalTools::getBuffer(0)[1]);
//	Communicating::getInstant(Communicating::COM1)->Send(4, AdditionalTools::getBuffer(0)[2]);
//	printf("m: ");
//	AdditionalTools::printfBuffer(0, 3);
//	printf("acc: ");
//	AdditionalTools::printfBuffer(1, 3);
}

void Output(){

//	printf("%g,%g,%g\n", Omega::getInstance(0)->getOmega(0),
//			Omega::getInstance(0)->getOmega(1),
//			Omega::getInstance(0)->getOmega(2));


	switch(Communicating::getInstant(Communicating::COM1)->getPrintType()){
		case 0:
			Communicating::getInstant(Communicating::COM1)->Send(0, (float)(MathTools::RadianToDegree(Quaternion::getInstance(0)->getEuler(0)) + Controlling::getInstant()->getRPYOffset(0)));
			Communicating::getInstant(Communicating::COM1)->Send(1, (float)(MathTools::RadianToDegree(Quaternion::getInstance(0)->getEuler(1)) + Controlling::getInstant()->getRPYOffset(1)));
			Communicating::getInstant(Communicating::COM1)->Send(2, (float)(MathTools::RadianToDegree(Quaternion::getInstance(0)->getEuler(2)) + Controlling::getInstant()->getRPYOffset(2)));
			break;
		case 1:
			Communicating::getInstant(Communicating::COM1)->Send(0, Controlling::getInstant()->getMotorTarget(0));
			Communicating::getInstant(Communicating::COM1)->Send(1, Controlling::getInstant()->getMotorTarget(1));
			Communicating::getInstant(Communicating::COM1)->Send(2, Controlling::getInstant()->getMotorTarget(2));
			Communicating::getInstant(Communicating::COM1)->Send(3, Controlling::getInstant()->getMotorTarget(3));
			break;
		case 2:
//			Communicating::getInstant(Communicating::COM1)->Send(8, (float)HMC5883L::getInstance(0)->getRawMagneticField(0));
//			Communicating::getInstant(Communicating::COM1)->Send(9, (float)HMC5883L::getInstance(0)->getRawMagneticField(1));
//			Communicating::getInstant(Communicating::COM1)->Send(10, (float)HMC5883L::getInstance(0)->getRawMagneticField(2));
			break;
		case 3:
			Communicating::getInstant(Communicating::COM1)->Send(0, (float)Acceleration::getInstance(0)->getAcc(0));
			Communicating::getInstant(Communicating::COM1)->Send(1, (float)Acceleration::getInstance(0)->getAcc(1));
			Communicating::getInstant(Communicating::COM1)->Send(2, (float)Acceleration::getInstance(0)->getAcc(2));
			break;
		case 4:
			Communicating::getInstant(Communicating::COM1)->Send(0, (float)Omega::getInstance(0)->getOmega(0));
			Communicating::getInstant(Communicating::COM1)->Send(1, (float)Omega::getInstance(0)->getOmega(1));
			Communicating::getInstant(Communicating::COM1)->Send(2, (float)Omega::getInstance(0)->getOmega(2));
			break;
		case 5:
			Communicating::getInstant(Communicating::COM1)->Send(0, MPU6050::getInstance(0)->getRawAcc(0));
			Communicating::getInstant(Communicating::COM1)->Send(1, MPU6050::getInstance(0)->getRawAcc(1));
			Communicating::getInstant(Communicating::COM1)->Send(2, MPU6050::getInstance(0)->getRawAcc(2));
			break;
		case 6:
			Communicating::getInstant(Communicating::COM1)->Send(0, (float)Controlling::getInstant()->getErrRPY(0));
			Communicating::getInstant(Communicating::COM1)->Send(1, (float)Controlling::getInstant()->getErrRPY(1));
			Communicating::getInstant(Communicating::COM1)->Send(2, (float)Controlling::getInstant()->getErrRPY(2));
			break;
		case 7:
			Communicating::getInstant(Communicating::COM1)->Send(4, (float)Controlling::getInstant()->watchDogCount);
			break;
		case 8:
			Communicating::getInstant(Communicating::COM1)->Send(0, (float)MathTools::RadianToDegree(Acceleration::getInstance(0)->getAngle(0)));
			Communicating::getInstant(Communicating::COM1)->Send(1, (float)MathTools::RadianToDegree(Acceleration::getInstance(0)->getAngle(1)));
			break;
		case 9:

			Communicating::getInstant(Communicating::COM1)->Send(0, (float)MathTools::RadianToDegree(Acceleration::getInstance(0)->getFilteredAngle(0)));
			Communicating::getInstant(Communicating::COM1)->Send(1, (float)MathTools::RadianToDegree(Acceleration::getInstance(0)->getFilteredAngle(1)));
			break;

	}
}

void PrintBattery(){
	Communicating::getInstant(Communicating::COM1)->Send(4, Battery::getInstance()->getBatteryLevel());
}

void Update(){
	MPU6050::getInstance(0)->Update();
	Omega::getInstance(0)->Update();
	Acceleration::getInstance(0)->Update();
	Quaternion::getInstance(0)->Update();
}

//void CompassUpdate(){
//	HMC5883L::getInstance()->Update();
//}
//
//void SE3Update(){
//	PX4FLOW::getInstance()->Update();
//	SE3::getInstance()->Update();
//}
//
void initUpdate(){
	MPU6050::getInstance(0)->Update();
	Omega::getInstance(0)->Update();
	Acceleration::getInstance(0)->Update();
//	HMC5883L::getInstance()->Update();
}
//
void ReceiveTask(){
	Communicating::getInstant(Communicating::COM1)->ReceivePoll();
}

void SendTask(){
	Communicating::getInstant(Communicating::COM1)->SendPoll();
}


void ControlTask(){
	Controlling::getInstant()->ControllingPoll();
}
//
//unsigned char Buffer[2048];
//unsigned char rfBuffer[2048];
//int BufferCount;
//int Timeout;
//int rfBufferCount;
//int rfTimeout;
//
//
//void RFTask(){
//	int uartLength = Usart::getInstance(USART1)->getBufferCount();
//	if(uartLength > 0){
//		Usart::getInstance(USART1)->Read(Buffer + BufferCount, uartLength);
//	}
//	BufferCount += uartLength;
//	if(BufferCount >= 8){
//		unsigned char B[9];
//		for(int i = 0; i < 8; i++){
//			B[i] = Buffer[i];
//		}
//		B[8] = '\0';
//		if(NRF905::getInstance()->Write("%s", B)){
//			BufferCount -= 8;
//			for(int i = 0; i < BufferCount; i++){
//				Buffer[i] = Buffer[i + 8];
//			}
//		}
//	}
//}
//
//void UartTask(){
//	int rfLength = NRF905::getInstance()->getBufferCount();
//	if(rfLength > 0){
//		NRF905::getInstance()->Read(rfBuffer + rfBufferCount, rfLength);
//	}
//	rfBufferCount += rfLength;
//
//	if(rfBufferCount >= 8){
//		unsigned char B[9];
//		for(int i = 0; i < 8; i++){
//			B[i] = rfBuffer[i];
//		}
//		B[8] = '\0';
//		rfBufferCount -= 8;
//		for(int i = 0; i < rfBufferCount; i++){
//			rfBuffer[i] = rfBuffer[i + 8];
//		}
////		Usart::getInstance(USART1)->Print("%s", B);
//		printf("%s", B);
//
//	}
//}

//int rfResetCount;
//int usartResetCount;
//
//void rfReset(){
//	if(rfResetCount++ > 10){
//		rfResetCount = 0;
//		Leds::getInstance()->Toggle(Leds::LED2);
//		NRF905::getInstance()->resetNRF905();
//		rfBufferCount = 0;
//		Usart::getInstance(USART1)->Print("RF RESETED\n");
//	}
//}
//
//void usartReset(){
//	if(usartResetCount++ > 10){
//		usartResetCount = 0;
//		Leds::getInstance()->Toggle(Leds::LED2);
//		//Usart::getInstance(USART3)->reset();
//		BufferCount = 0;
//		Usart::getInstance(USART1)->Print("USART RESETED\n");
//	}
//}
//
//void Test(){
//	//Usart::getInstance(USART1)->Print("Plot:%g,%g,%g\n", Quaternion::getInstance()->getEuler(0), Quaternion::getInstance()->getEuler(1), Quaternion::getInstance()->getEuler(2));
//	//Usart::getInstance(USART1)->Print("BatteryLevel:%g\n", Battery::getInstance()->getBatteryLevel());
//	//Usart::getInstance(USART1)->Print("%d\n", Ticks::getInstance()->getTicks());
////	Usart::getInstance(USART1)->Print("Plot:%g,%g,%g,%g\n", Controlling::getInstant()->getFzPWM() * 3600 / K / CT);//,
////			Controlling::getInstant()->getRotorPWM(1),
////			Controlling::getInstant()->getRotorPWM(2),
////			Controlling::getInstant()->getRotorPWM(3));
//}
//
//int main1(){
//
//	Delay::DelayMS(100);
//	Task mTask;
//	Leds mLeds;
//
//	Usart mUsart1(USART1, 256000);
//	uint8_t rxAddress[4] = {0x01, 0x09, 0x08, 0x07};
//	uint8_t txAddress[4] = {0x01, 0x00, 0x00, 0x08};
//	NRF905 mNRF905(8, NRF905::FREQ_433M, NRF905::PWR_POS_10dBM, NRF905::RX_NORMAL_PWR, NRF905::NO_RETRAN, rxAddress, txAddress, 8, 8);
//
//	BufferCount = 0;
//	rfBufferCount = 0;
//	mLeds.Blink(100, Leds::LED1, true);
//	mTask.Attach(10, 0, UartTask, true, -1);
//	mTask.Attach(10, 7, RFTask, true, -1);
//	mTask.Run();
//}
//
//void SonicUpdateTask(){
//	Sonic::getInstance()->Update();
//}
//
//void Sampling(){
////	Usart::getInstance(USART1)->Print("$,%g,%g,%g,%g,%g,%g\n", Quaternion::getInstance()->temp1[0],
////			Quaternion::getInstance()->temp1[1],
////			Quaternion::getInstance()->temp1[2],
////			Quaternion::getInstance()->temp2[0],
////			Quaternion::getInstance()->temp2[1],
////			Quaternion::getInstance()->temp2[2]);
//}
//
//void Testing(){
//	Usart::getInstance(USART1)->Print("%d\n", Ticks::getInstance()->getTicks());
//}
//

void SonicUpdate(){
	Sonic::getInstance()->Update();
}

void PrintSonic(){
	Usart::getInstance(USART1)->Print("%g\n", Sonic::getInstance()->getDistance());
}

int main(){

	Delay::DelayMS(100);

	Task* mTask = new Task(true);
	Leds* mLeds = new Leds();
	Battery* mBattery = new Battery();
	Controlling* mControlling = new Controlling();
	Usart* mUsart1 = new Usart(USART1, 115200);
	Usart::setPrintUsart(USART1);
	mUsart1->Print("Started\n");
	Sonic* sonic = new Sonic();
	mLeds->Blink(100, Leds::LED1, true);
	mTask->Attach(100, 0, SonicUpdate, true);
//	printf("%g\n", mBattery->getBatteryLevel());
//	Communicating* mCommunicating = new Communicating(Communicating::COM1, USART1, false);
//
//	I2C* mI2C2 = new I2C(I2C2, I2C::SPEED_400K);
//	MPU6050* mMPU6050 = new MPU6050(0, mI2C2);
//	Acceleration* mAcceleration = new Acceleration(0);
//	Omega* mOmega = new Omega(0);
////	HMC5883L mHMC5883L(66);
////
//	mTask->Attach(2, 0, initUpdate, false, 500, false);
//	mTask->Run();
//
//	Quaternion* mQuaternion = new Quaternion(0, 0.002f);
//	mQuaternion->resetQuaternion();
//
//	if(mBattery->getBatteryLevel() > 3.7f){
//		mLeds->Blink(100, Leds::LED1, true);
//		mLeds->Blink(100, Leds::LED2, true);
//		mLeds->Blink(100, Leds::LED3, true);
//		mLeds->Blink(100, Leds::LED4, true);
//	}
//	else{
//		mLeds->LedsControl(Leds::LED1, false);
//		mLeds->LedsControl(Leds::LED2, false);
//		mLeds->LedsControl(Leds::LED3, false);
//		mLeds->LedsControl(Leds::LED4, false);
//	}
//
//	mTask->Attach(2, 0, Update, true);
////	mTask->Attach(66, 0, CompassUpdate, true);
//////	mTask->Attach(60, 0, SonicUpdateTask, true, -1);
//	mTask->Attach(2, 1, ControlTask, true);
//////	mTask->Attach(8, 3, SE3Update, true, -1);
//	mTask->Attach(5, 0, ReceiveTask, true);
//	mTask->Attach(10, 1, SendTask, true);
//	mTask->Attach(20, 11, Output, true);
//	mTask->Attach(500, 23, printParams, true);
////	mTask->Attach(1000, 110, PrintBattery, true);
////	mTask->Attach(10, 0, Sampling, true);

	mTask->Run();

	return 0;
}

void HardFault_Handler(){
	while(true){
		printf("HardFault\n");
		Delay::DelayMS(100);
	}
}
