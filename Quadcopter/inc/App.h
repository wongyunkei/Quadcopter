/*
 * App.h
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#ifndef APP_H_
#define APP_H_

#include <stm32f4xx.h>
#include <inttypes.h>
#include <stdio.h>
#include <Delay.h>
#include <Config.h>
#include <Ticks.h>
#include <Task.h>
#include <Led.h>
#include <UART.h>
#include <Sonic.h>
#include <Kalman.h>
//#include <Battery.h>
//#include <stm32f4xx_gpio.h>
//#include <stm32f4xx_rcc.h>
//#include <stm32f4xx_usart.h>
//#include <stdlib.h>
//#include <string.h>

//#include <I2C.h>
//#include <MPU6050.h>
//#include <Controlling.h>
//#include <Acceleration.h>
//#include <Omega.h>
//#include <Quaternion.h>
//#include <MathTools.h>
//#include <AdditionalTools.h>
////#include <Vector.h>
//#include <Communicating.h>
////#include <NRF905.h>
////#include <stm32f4xx_dma.h>
//#include <Battery.h>
////#include <PWM.h>
////#include <PhasesMonitoring.h>
////#include <stm32f4xx_it.h>
////#include <Buzzer.h>
////#include <PX4FLOW.h>
////#include <SE3.h>
////#include <Kalman.h>
////#include <HMC5883L.h>

namespace System{
	class App{
		public:
			static App* mApp;
			Ticks* mTicks;
			Task* mTask;
			Config* mConfig;
			Led* mLed1;
			Led* mLed2;
			Led* mLed3;
			Led* mLed4;
			UART* mUART1;
			UART* mUART3;
			UART* mUART4;
			UART* mUART5;
			Communicating* mCommunicating1;
			Communicating* mCommunicating2;
			Communicating* mCommunicating3;
			Communicating* mCommunicating4;
			Sonic* mSonic1;
			Sonic* mSonic2;
			Sonic* mSonic3;
			Sonic* mSonic4;
			Kalman* mSonicKalman;
			App();
		private:
	};
};

using namespace System;

#endif /* APP_H_ */
