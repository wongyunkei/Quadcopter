/*
 * App.h
 *
 *  Created on: 2015�~11��27��
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
#include <ExternalInterrupt.h>
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

namespace System{
	class App{
		public:
			static App* mApp;
			Ticks* mTicks;
			Task* mTask;
			Config* mConfig;
			ExternalInterrupt* mExti[16];
			Led* mLed1;
			Led* mLed2;
			Led* mLed3;
			Led* mLed4;

			Led* mSonicTrigger[16];

			Led* mGPIO1;
			Led* mGPIO2;
			Led* mGPIO3;
			Led* mGPIO4;
			Led* mGPIO5;
			Led* mGPIO6;
			Led* mGPIO7;
			Led* mGPIO8;
			UART* mUART1;
			UART* mUART2;
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
			Sonic* mSonic5;
			Sonic* mSonic6;
			Sonic* mSonic7;
			Sonic* mSonic8;
			Sonic* mSonic9;
			Sonic* mSonic10;
			Sonic* mSonic11;
			Sonic* mSonic12;
			Sonic* mSonic13;
			Sonic* mSonic14;
			Sonic* mSonic15;
			Sonic* mSonic16;
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
			Encoder* mEncoder4;
			Encoder* mEncoder5;
			Encoder* mEncoder6;
			Localization* mLocalization;
			EncoderYaw* mEncoderYaw;
			App();
			int PathState;
			int PeriodicCmd;
			float PeriodicData;
			int PeriodicCmd2;
			float PeriodicData2;
			PT currentPT;
			PT nextPT;
			bool trigger;
			bool arrived;

			Pid* Motor1PID;
			Pid* Motor2PID;
			Pid* Motor3PID;
			float Motor1Target;
			float Motor2Target;
			float Motor3Target;
			int IsCal1;
			int IsCal2;
			int IsCal3;
			bool ControlStart;
		private:
	};
};

#endif /* APP_H_ */
