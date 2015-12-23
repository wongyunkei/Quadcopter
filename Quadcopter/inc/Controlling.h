/*
 * Controlling.h
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#ifndef CONTROLLING_H_
#define CONTROLLING_H_

#include <PWM.h>
#include <App.h>
#include <Communicating.h>
#include <Quaternion.h>
#include <math.h>
#include <MathTools.h>
#include <stdio.h>
#include <Pid.h>
#include <PWM.h>
#include <Omega.h>
#include <Task.h>
#include <Acceleration.h>
#include <AdditionalTools.h>
#include <Delay.h>
#include <Led.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;
using Eigen::Vector4f;

using namespace Math;

namespace Control{

	class Controlling{
		public:
			Controlling(PWM* mPWM);
			void ControllingPoll();
			void Starting();
			void Stopping();
			float getRollTarget();
			float getPitchTarget();
			float getYawTarget();
			void setRollTarget(float roll);
			void setPitchTarget(float pitch);
			void setYawTarget(float yaw);
			float getRollOffset();
			float getPitchOffset();
			float getYawOffset();
			void setRollOffset(float roll);
			void setPitchOffset(float pitch);
			void setYawOffset(float yaw);
			void clearWatchDogCount();
			void setStart(bool);
			bool getStart();
			void setStarting(bool);
			bool getStarting();
			void setStopping(bool);
			bool getStopping();
			void StopAllMotors();
			PWM* _mPWM;
			Pid* RollPid;
			Pid* PitchPid;
			Pid* YawPid;
			Pid* KdRollPid;
			Pid* KdPitchPid;
			Pid* KdYawPid;
			float maxLift;
			float minLift;
			float initLift;
			float leadingLift;
			float Lift;
			int watchDogCount;
			float RollOffset;
			float PitchOffset;
			float YawOffset;
			int startCount;
			int StoppingDelayCount;

		private:
			int WatchDogLimit;
			float RollTarget;
			float PitchTarget;
			float YawTarget;
			bool started;
			bool starting;
			bool stopping;
	};
};

#endif /* CONTROLLING_H_ */
