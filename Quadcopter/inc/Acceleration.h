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

			Acceleration(int index);
			static Acceleration* getInstance(int index);
			void Update();
			float getAcc(int channel);
			float getVel(int channel);
			float getPos(int channel);
			void setAcc(int channel, float value);
			float getRawAcc(int channel);
			float getAngle(int channel);
			float getFilteredAngle(int channel);
			MovingWindowAverageFilter* getMovingAverageFilter(int channel);
			bool getIsValided();

		private:
			bool isValided;
			int DevIndex;
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
