/*
 * Pid.h
 *
 *  Created on: 2014¦~11¤ë8¤é
 *      Author: YunKei
 */

#ifndef PID_H_
#define PID_H_

namespace Math{

	class Pid{

		public:

			Pid(int, float, float, float, float, float);
			static Pid* getInstance(int);
			float setPid(float, float, float);
			float getPid(int);
			float pid(float, float);
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
