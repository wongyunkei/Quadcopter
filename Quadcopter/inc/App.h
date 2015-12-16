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
#include <MathTools.h>
#include <stdio.h>
#include <Delay.h>
#include <Config.h>
#include <Ticks.h>
#include <Task.h>
#include <Led.h>
#include <UART.h>
#include <PWM.h>
#include <ADConverter.h>
#include <Sonic.h>
#include <I2C.h>
#include <Kalman.h>
#include <MPU6050.h>
#include <Acceleration.h>
#include <Omega.h>
#include <Quaternion.h>

using namespace Time;
using namespace Math;
using namespace Sensors;
using namespace Inertia;

namespace Sensors{
	class MPU6050;
};

namespace Inertia{
	class Acceleration;
};

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
			Led* mGPIO1;
			UART* mUART1;
			UART* mUART3;
			UART* mUART4;
			UART* mUART5;
			Communicating* mCommunicating1;
			Communicating* mCommunicating2;
			Communicating* mCommunicating3;
			Communicating* mCommunicating4;
			PWM* mPWM;
			ADConverter* mADC;
			Sonic* mSonic1;
			Sonic* mSonic2;
			Sonic* mSonic3;
			Sonic* mSonic4;
			Kalman* mADCKalman;
			I2C* mI2C1;
			I2C* mI2C2;
			MPU6050* mMPU6050;
			Acceleration* mAcceleration;
			Omega* mOmega;
			Quaternion* mQuaternion;
			App();
			double initVolt;
		private:
	};
};

#endif /* APP_H_ */
