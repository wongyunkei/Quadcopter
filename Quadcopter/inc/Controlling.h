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

namespace Control{

	class Controlling{
		public:
			#define WATCHDOGCOUNT_LIMIT	1000
			#define JX	0.047387
			#define JY	0.047387
			#define JZ	0.07438
			#define __M	1.68
			#define __R	0.275
			#define CT	0.12
			#define CD	0.05
			#define __K	23
			#define INIT_PWM		3500.0f
			#define INIT_RPM		3500.0f
			#define MAX_RPM			7000.0f
			Controlling();
			static Controlling* getInstant();
			void ControllingPoll();
			void Starting();
			void Stopping();
			void setTarget(int, double);
			double getThrust(int);
			double getOffset(int);
			void setOffset(int, double);
			void setRPYOffset(int, double);
			double getRPYOffset(int);
			void clearWatchDogCount();
			double getErrRPY(int);
			double getInitPWM();
			void setInitPWM(double);
			double getInitRPM();
			void setInitRPM(double);
			void setRotorPWM(int, double);
			double getRotorPWM(int);
			void setStart(bool);
			bool getStart();
			void setStarting(bool);
			bool getStarting();
			void setStopping(bool);
			bool getStopping();
			double getFzPWM();
			void setFzPWM(double);
			double getPreFzPWM();
			void setPreFzPWM(double);
			double getTarget(int);
			void MotorControl(int index, float value);
			float RPM2PWM(int index, float rpm);
			double getMotorRPM(int index);
			void setMotorRPM(int index, double value);
			double getMotorValue(int index);
			void setMotorValue(int index, double value);
			void setMotorTarget(int index, double value);
			double getMotorTarget(int index);
			int watchDogCount;
		private:
			PWM* ControlPWM;
			Fuzzy* FuzzyRPY[3];
			Pid* RPYPid[3];
			Pid* MotorPid[4];
			double MotorTarget[4];
			double MotorRPM[4];
			double MotorValue[4];
			Pid* HightPid;
			double preFzPWM;
			int HightPidDelayCount;
			Pid* D_RPYPid[3];
			double RPYOffset[3];
			double errRPY[3];
			double target[4];
			double thrust[4];
			double offset[4];
			double rotorPWM[4];
			bool started;
			bool starting;
			bool stopping;
			double initPWM;
			double initRPM;
			double FzPWM;
			double cosRollcosPitch;
			double** fuzzyTable;

	};
};

using namespace Control;

#endif /* CONTROLLING_H_ */
