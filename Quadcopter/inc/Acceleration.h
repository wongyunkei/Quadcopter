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
#include <Vector.h>

namespace Sensors{

	class Acceleration{

		public:

			#define GRAVITY		9.80665f

			Acceleration();
			static Acceleration* getInstance();
			void Update();
			float getAcc(int);
			float getVel(int);
			float getPos(int);
			void setAcc(int, float);
			float getRawAcc(int);
			float getAngle(int);
			float getFilteredAngle(int);
			MovingWindowAverageFilter* getMovingAverageFilter(int index);
			bool getIsValided();

		private:
			bool isValided;
			float Acc[3];
			float Pos[3];
			float Vel[3];
			float RawAcc[3];
			Kalman* AccKalman[3];
			MovingWindowAverageFilter* accMovingAverage[3];
	};
};

using namespace Sensors;

#endif /* ACCELERATION_H_ */
