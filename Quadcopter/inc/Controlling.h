/*
 * Controlling.h
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#ifndef CONTROLLING_H_
#define CONTROLLING_H_

#include <PWM.h>
#include <Pid.h>
#include <Acceleration.h>
#include <Fuzzy.h>

using namespace Math;

namespace Control{

	class Controlling{
		public:
			#define WATCHDOGCOUNT_LIMIT	800
			#define JX	0.047387
			#define JY	0.047387
			#define JZ	0.07438
			#define __M	1.68
			#define __R	0.275
			#define CT	0.12
			#define CD	0.05
			#define __K	23
			#define INIT_PWM		2500.0f
			#define INIT_RPM		0.0f
			#define MIN_LIFT		2500.0f
			#define MAX_LIFT		10000.0f
			#define LANDING_MAX_LIFT		5000.0f
			#define MAX_RPM			5000.0f
			Controlling();
			static Controlling* getInstant();
			void ControllingPoll();
			void Starting();
			void Stopping();
			void setTarget(int, float);
			float getThrust(int);
			float getOffset(int);
			void setOffset(int, float);
			void setRPYOffset(int, float);
			float getRPYOffset(int);
			void clearWatchDogCount();
			float getErrRPY(int);
			float getInitPWM();
			void setInitPWM(float);
			float getInitRPM();
			void setInitRPM(float);
			void setRotorPWM(int, float);
			float getRotorPWM(int);
			void setStart(bool);
			bool getStart();
			void setStarting(bool);
			bool getStarting();
			void setStopping(bool);
			bool getStopping();
			float getFzPWM();
			void setFzPWM(float);
			float getPreFzPWM();
			void setPreFzPWM(float);
			float getTarget(int);
			void MotorControl(int index, float value);
			float RPM2PWM(int index, float rpm);
			float getMotorRPM(int index);
			void setMotorRPM(int index, float value);
			float getMotorValue(int index);
			void setMotorValue(int index, float value);
			void setMotorTarget(int index, float value);
			float getMotorTarget(int index);
			float getLift();
			void setLift(float value);
			void setMaxLift(float value);
			void setMinLift(float value);
			int watchDogCount;

		private:
			PWM* ControlPWM;
			Fuzzy* FuzzyRPY[3];
			Pid* RPYPid[3];
			Pid* XYPid[2];
			Pid* HightPid;
			Pid* MotorPid[4];
			float MotorTarget[4];
			float MotorRPM[4];
			float MotorValue[4];
			float preFzPWM;
			int HightPidDelayCount;
			int XYPidDelayCount;
			Pid* D_RPYPid[3];
			float RPYOffset[3];
			float Lift;
			float errRPY[3];
			float errXY[2];
			float target[4];
			float thrust[4];
			float offset[4];
			float rotorPWM[4];
			bool started;
			bool starting;
			bool stopping;
			float initPWM;
			float minLift;
			float maxLift;
			float initRPM;
			float FzPWM;
			float cosRollcosPitch;
			float** fuzzyTable;

	};
};

#endif /* CONTROLLING_H_ */
