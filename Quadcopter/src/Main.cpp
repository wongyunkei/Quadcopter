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
#include <Spi.h>
#include <I2C.h>
#include <MPU6050.h>
#include <Acceleration.h>
#include <Omega.h>
#include <Quaternion.h>
#include <MathTools.h>
#include <Vector.h>
#include <Communicating.h>
#include <Controlling.h>
#include <NRF905.h>
#include <stm32f4xx_dma.h>
#include <Battery.h>
#include <PWM.h>
#include <Sonic.h>
#include <PhasesMonitoring.h>
#include <Math.h>

void TestMotorTask(){
	static double pwm;
	Usart::getInstance(USART1)->Print("%g,%g,%g,%g,%g\n", pwm,
			PhasesMonitoring::getInstance()->getRPM(0),
			PhasesMonitoring::getInstance()->getRPM(1),
			PhasesMonitoring::getInstance()->getRPM(2),
			PhasesMonitoring::getInstance()->getRPM(3));
	pwm += 100;
	if(pwm <= 6000){
		for(int i = 0; i < 4; i++){
			PWM::getInstant()->Control(i, pwm);
		}
	}else{
		for(int i = 0; i < 4; i++){
			PWM::getInstant()->Control(i, 0);
		}
	}
}

void InterruptPrint(){

	Usart::getInstance(USART1)->Print("T:%d\n", PhasesMonitoring::getInstance()->getInterruptCount());

	PhasesMonitoring::getInstance()->setInterruptCount(0);
}

int printfCount;

void RFOutput(){
	switch(Communicating::getInstant()->getPrintType()){
		case 0:
			Communicating::getInstant()->RFSend(0, (float)(MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(0))));
			Communicating::getInstant()->RFSend(1, (float)(MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(1))));
			Communicating::getInstant()->RFSend(2, (float)(MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(2))));
			break;
		case 1:
			Communicating::getInstant()->RFSend(0, (float)PhasesMonitoring::getInstance()->getRPM(0));
			Communicating::getInstant()->RFSend(1, (float)PhasesMonitoring::getInstance()->getRPM(1));
			Communicating::getInstant()->RFSend(2, (float)PhasesMonitoring::getInstance()->getRPM(2));
			Communicating::getInstant()->RFSend(3, (float)PhasesMonitoring::getInstance()->getRPM(3));
			break;
		case 2:
			Communicating::getInstant()->RFSend(0, (float)Acceleration::getInstance()->getAcc(0));
			Communicating::getInstant()->RFSend(1, (float)Acceleration::getInstance()->getAcc(1));
			Communicating::getInstant()->RFSend(2, (float)Acceleration::getInstance()->getAcc(2));
			break;
		case 3:
			Communicating::getInstant()->RFSend(0, (float)Acceleration::getInstance()->getMovingAverageFilter(0)->getAverage());
			Communicating::getInstant()->RFSend(1, (float)Acceleration::getInstance()->getMovingAverageFilter(1)->getAverage());
			Communicating::getInstant()->RFSend(2, (float)Acceleration::getInstance()->getMovingAverageFilter(2)->getAverage());
			break;
		case 4:
			Communicating::getInstant()->RFSend(0, (float)Omega::getInstance()->getOmega(0));
			Communicating::getInstant()->RFSend(1, (float)Omega::getInstance()->getOmega(1));
			Communicating::getInstant()->RFSend(2, (float)Omega::getInstance()->getOmega(2));
			break;
		case 5:
			Communicating::getInstant()->RFSend(0, (float)Acceleration::getInstance()->getFilteredAngle(0));
			Communicating::getInstant()->RFSend(1, (float)Acceleration::getInstance()->getFilteredAngle(1));
			break;
	}
}

void BatteryPrint(){
	Communicating::getInstant()->RFSend(4, (float)Battery::getInstance()->getBatteryLevel());
}

void Output(){

//	Usart::getInstance(USART1)->Print("$,%g,%g,%g,%g,%g\n", MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(0)),
//			MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(1)),
//			MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(2)),
//			MathTools::RadianToDegree(Acceleration::getInstance()->getAngle(0)),
//			MathTools::RadianToDegree(Acceleration::getInstance()->getAngle(1)));


//	Usart::getInstance(USART1)->Print("$,%g,%g\n", MathTools::RadianToDegree(Acceleration::getInstance()->getAngle(0)),
//			MathTools::RadianToDegree(Acceleration::getInstance()->getAngle(1)));


	switch(Communicating::getInstant()->getPrintType()){
		case 0:
			Usart::getInstance(USART1)->Print("$,%g,%g,%g\n", MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(0)),
					MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(1)),
					MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(2)));
			break;
		case 1:
			Usart::getInstance(USART1)->Print("$,%g,%g,%g,%g\n", PhasesMonitoring::getInstance()->getRPM(0),
					PhasesMonitoring::getInstance()->getRPM(1),
					PhasesMonitoring::getInstance()->getRPM(2),
					PhasesMonitoring::getInstance()->getRPM(3));
			break;
		case 2:
			Usart::getInstance(USART1)->Print("$,%g,%g,%g\n", Acceleration::getInstance()->getAcc(0),
					Acceleration::getInstance()->getAcc(1),
					Acceleration::getInstance()->getAcc(2));
			break;
		case 3:
			Usart::getInstance(USART1)->Print("$,%g,%g,%g\n", Acceleration::getInstance()->getAcc(0),
					Acceleration::getInstance()->getMovingAverageFilter(1)->getAverage(),
					Acceleration::getInstance()->getMovingAverageFilter(2)->getAverage());
			break;
		case 4:
				Usart::getInstance(USART1)->Print("$,%g,%g,%g\n", Omega::getInstance()->getOmega(0),
				Omega::getInstance()->getOmega(1),
				Omega::getInstance()->getOmega(2));
			break;
	}


//	printf("$,%g,%g,%g,", MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(0)),
//			MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(1)),
//			MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(2)));
//
//	printf("%g,%g,%g,", Omega::getInstance()->getOmega(0),
//				Omega::getInstance()->getOmega(1),
//			Omega::getInstance()->getOmega(2));
//
//	printf("%g,%g,%g\n", Acceleration::getInstance()->getAcc(0),
//				Acceleration::getInstance()->getAcc(1),
//				Acceleration::getInstance()->getAcc(2));
}

void Print(){

	float E[4];

	switch(Communicating::getInstant()->getPrintType()){
		case 0:
			for(int i = 0; i < 3; i++){
				E[i] = MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(i)) + Controlling::getInstant()->getRPYOffset(i);
			}
			E[3] = Communicating::getInstant()->getCmdData();
			break;
		case 1:
			for(int i = 0; i < 3; i++){
				E[i] = Acceleration::getInstance()->getAcc(i);
			}
			E[3] = Communicating::getInstant()->getCmdData();
			break;
		case 2:
			for(int i = 0; i < 3; i++){
				E[i] = Omega::getInstance()->getOmega(i);
			}
			E[3] = Communicating::getInstant()->getCmdData();
			break;
		case 4:
			for(int i = 0; i < 4; i++){
				E[i] = Controlling::getInstant()->getRotorPWM(i);
			}
			break;
		case 5:
			for(int i = 0; i < 3; i++){
				E[i] = Controlling::getInstant()->getRPYOffset(i);
			}
			E[3] = Communicating::getInstant()->getCmdData();
			break;
		case 6:
			for(int i = 0; i < 3; i++){
				E[i] = Sonic::getInstance()->getDistance();
			}
			E[3] = Communicating::getInstant()->getCmdData();
			break;
	}
//	Communicating::getInstant()->RFSend(0, E);
}

void Update(){
	MPU6050::getInstance()->Update();
	Omega::getInstance()->Update();
	Acceleration::getInstance()->Update();
	Quaternion::getInstance()->Update();
}

void initUpdate(){
	MPU6050::getInstance()->Update();
	Omega::getInstance()->Update();
	Acceleration::getInstance()->Update();
}

void ReceiveTask(){

	Communicating::getInstant()->ReceivePoll();
}

void SendTask(){

	Communicating::getInstant()->SendPoll();
}

void ControlTask(){
	Controlling::getInstant()->ControllingPoll();
}

unsigned char Buffer[2048];
unsigned char rfBuffer[2048];
int BufferCount;
int Timeout;
int rfBufferCount;
int rfTimeout;


void RFTask(){
	int uartLength = Usart::getInstance(USART1)->getBufferCount();
	if(uartLength > 0){
		Usart::getInstance(USART1)->Read(Buffer + BufferCount, uartLength);
	}
	BufferCount += uartLength;
	if(BufferCount >= 8){
		unsigned char B[9];
		for(int i = 0; i < 8; i++){
			B[i] = Buffer[i];
		}
		B[8] = '\0';
		if(NRF905::getInstance()->Write("%s", B)){
			BufferCount -= 8;
			for(int i = 0; i < BufferCount; i++){
				Buffer[i] = Buffer[i + 8];
			}
		}
	}
}

void UartTask(){
	int rfLength = NRF905::getInstance()->getBufferCount();
	if(rfLength > 0){
		NRF905::getInstance()->Read(rfBuffer + rfBufferCount, rfLength);
	}
	rfBufferCount += rfLength;

	if(rfBufferCount >= 8){
		unsigned char B[9];
		for(int i = 0; i < 8; i++){
			B[i] = rfBuffer[i];
		}
		B[8] = '\0';
		rfBufferCount -= 8;
		for(int i = 0; i < rfBufferCount; i++){
			rfBuffer[i] = rfBuffer[i + 8];
		}
//		Usart::getInstance(USART1)->Print("%s", B);
		printf("%s", B);

	}
}

int rfResetCount;
int usartResetCount;

void rfReset(){
	if(rfResetCount++ > 10){
		rfResetCount = 0;
		Leds::getInstance()->Toggle(Leds::LED2);
		NRF905::getInstance()->resetNRF905();
		rfBufferCount = 0;
		Usart::getInstance(USART1)->Print("RF RESETED\n");
	}
}

void usartReset(){
	if(usartResetCount++ > 10){
		usartResetCount = 0;
		Leds::getInstance()->Toggle(Leds::LED2);
		//Usart::getInstance(USART3)->reset();
		BufferCount = 0;
		Usart::getInstance(USART1)->Print("USART RESETED\n");
	}
}

void Test(){
	//Usart::getInstance(USART1)->Print("Plot:%g,%g,%g\n", Quaternion::getInstance()->getEuler(0), Quaternion::getInstance()->getEuler(1), Quaternion::getInstance()->getEuler(2));
	//Usart::getInstance(USART1)->Print("BatteryLevel:%g\n", Battery::getInstance()->getBatteryLevel());
	//Usart::getInstance(USART1)->Print("%d\n", Ticks::getInstance()->getTicks());
	Usart::getInstance(USART1)->Print("Plot:%g,%g,%g,%g\n", Controlling::getInstant()->getFzPWM() * 3600 / K / CT);//,
//			Controlling::getInstant()->getRotorPWM(1),
//			Controlling::getInstant()->getRotorPWM(2),
//			Controlling::getInstant()->getRotorPWM(3));
}

int main1(){

	Delay::DelayMS(100);
	Task mTask;
	Leds mLeds;

	Usart mUsart1(USART1, 256000);
	uint8_t rxAddress[4] = {0x01, 0x09, 0x08, 0x07};
	uint8_t txAddress[4] = {0x01, 0x00, 0x00, 0x08};
	NRF905 mNRF905(8, NRF905::FREQ_433M, NRF905::PWR_POS_10dBM, NRF905::RX_NORMAL_PWR, NRF905::NO_RETRAN, rxAddress, txAddress, 8, 8);

	BufferCount = 0;
	rfBufferCount = 0;
	mLeds.Blink(100, Leds::LED1, true);
	mTask.Attach(20, 0, UartTask, true, -1);
	mTask.Attach(20, 0, RFTask, true, -1);
	mTask.Run();
}

void SonicUpdateTask(){
	Sonic::getInstance()->Update();
}

void Sampling(){
	Usart::getInstance(USART1)->Print("$,%g,%g,%g,%g,%g,%g\n", Quaternion::getInstance()->temp1[0],
			Quaternion::getInstance()->temp1[1],
			Quaternion::getInstance()->temp1[2],
			Quaternion::getInstance()->temp2[0],
			Quaternion::getInstance()->temp2[1],
			Quaternion::getInstance()->temp2[2]);
}

void Testing(){
	Usart::getInstance(USART1)->Print("%d\n", Ticks::getInstance()->getTicks());
}

int main(){

	Delay::DelayMS(100);
	Task mTask;
	Leds mLeds;
	Battery mBattery;

	Controlling mControlling;
//	Usart mUsart1(USART1, 256000);
	Usart mUsart1(USART1, 115200);
	mTask.Attach(100, 0, Testing, true, -1);
	mTask.Run();
	uint8_t txAddress[4] = {0x01, 0x09, 0x08, 0x07};
	uint8_t rxAddress[4] = {0x01, 0x00, 0x00, 0x08};
	NRF905 mNRF905(8, NRF905::FREQ_433M, NRF905::PWR_POS_10dBM, NRF905::RX_NORMAL_PWR, NRF905::NO_RETRAN, rxAddress, txAddress, 8, 8);
	Communicating mCommunicating(USART1, true);
	I2C mI2C2(I2C2, I2C::SPEED_400K);
	MPU6050 mMPU6050(0.002);
	Acceleration mAcceleration;
	Omega mOmega;
	Quaternion mQuaternion(0.002);
	PhasesMonitoring mPhasesMonitoring;

	Delay::DelayMS(100);
	mLeds.Blink(100, Leds::LED1, true);
	mTask.Attach(2, 0, initUpdate, false, 128);
	mTask.Run();
//	Sonic mSonic;

	Usart::getInstance(USART1)->Print("V:%g\n", Battery::getInstance()->getBatteryLevel());
	printfCount = 0;
	mTask.Attach(2, 0, Update, true, -1);
//	mTask.Attach(60, 0, SonicUpdateTask, true, -1);
	mTask.Attach(2, 0, ControlTask, true, -1);
	mTask.Attach(20, 0, ReceiveTask, true, -1);
	mTask.Attach(20, 10, SendTask, true, -1);
//	mTask.Attach(100, 50, Output, true, -1);
	mTask.Attach(100, 50, RFOutput, true, -1);
//	mTask.Attach(100, 50, Sampling, true, -1);
	mTask.Attach(5000, 120, BatteryPrint, true, -1);
//	mTask.Attach(1000, 0, InterruptPrint, true, -1);
//	mTask.Attach(2000, 0, Test, true, -1);
	mTask.Run();

	return 0;
}
