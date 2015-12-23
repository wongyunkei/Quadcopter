/*
 * Pid.h
 *
 *  Created on: 2014¦~11¤ë8¤é
 *      Author: YunKei
 */

#ifndef PID_H_
#define PID_H_

#include <Eigen/Eigen>

namespace Math{

	class Pid{

		public:

			Pid(float kp, float ki, float kd, float integralLimit, float t);
			void setKp(float kp);
			void setKi(float ki);
			void setKd(float kd);
			float getKp();
			float getKi();
			float getKd();

			float pid(float target, float current);
			void clear();
			float getIntegral();

		private:

			float Kp, Ki, Kd;
			float Integral;
			float IntegralLimit;
			float PreErr;
			float PreTimeStamp;
			float Period;
			float DefaultPeriod;

	};
};

#endif /* PID_H_ */
