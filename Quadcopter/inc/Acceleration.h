/*
 * Acceleration.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef ACCELERATION_H_
#define ACCELERATION_H_

#include <Kalman.h>
#include <MovingWindowAverageFilter.h>

namespace Sensors{

	class Acceleration{

		public:

			#define GRAVITY		9.80665

			Acceleration();
			static Acceleration* getInstance();
			void Update();
			double getAcc(int);
			void setAcc(int, double);
			double getRawAcc(int);
			double getAngle(int);
			double getFilteredAngle(int);
			MovingWindowAverageFilter* getMovingAverageFilter(int index);

		private:
			double Acc[3];
			double RawAcc[3];
			Kalman* AccKalman[3];
			MovingWindowAverageFilter* accMovingAverage[3];
	};
};

using namespace Sensors;

#endif /* ACCELERATION_H_ */
