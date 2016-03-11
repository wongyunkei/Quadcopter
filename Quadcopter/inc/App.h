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
#include <Communicating.h>
#include <Ticks.h>
#include <Task.h>
#include <Led.h>
#include <UART.h>
#include <PWM.h>
#include <ADConverter.h>
#include <Sonic.h>
#include <Spi.h>
#include <I2C.h>
#include <Kalman.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <Acceleration.h>
#include <Omega.h>
#include <Compass.h>
#include <Quaternion.h>
#include <AdditionalTools.h>
#include <Controlling.h>
#include <MovingWindowAverageFilter.h>
#include <Encoder.h>
#include <Localization.h>
#include <EncoderYaw.h>
#include <Eigen/Eigen>

using namespace Time;
using namespace Math;
using namespace Sensors;
using namespace Inertia;
using namespace Control;
using namespace Communication;
using namespace Debug;

namespace Debug{
	class Led;
};

namespace System{
	class Config;
};

namespace Control{
	class Controlling;
};

namespace Communication{
	class Communicating;
};

namespace Sensors{
	class MPU6050;
	class HMC5883L;
	class Encoder;
};

namespace Inertia{
	class Acceleration;
	class Omega;
	class Compass;
	class EncoderYaw;
};

namespace Math{
	class Quaternion;
	class Localization;
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
			Spi* mSpi1;
			Spi* mSpi2;
			I2C* mI2C1;
			I2C* mI2C2;
			MPU6050* mMPU6050;
			HMC5883L* mHMC5883L;
			Acceleration* mAcceleration;
			Omega* mOmega;
			Compass* mCompass;
			Quaternion* mQuaternion;
			Controlling* mControlling;
			MovingWindowAverageFilter* mADCFilter;
			Encoder* mEncoder1;
			Encoder* mEncoder2;
			Encoder* mEncoder3;
			Localization* mLocalization;
			EncoderYaw* mEncoderYaw;
			App();
		private:
	};
};

#endif /* APP_H_ */
