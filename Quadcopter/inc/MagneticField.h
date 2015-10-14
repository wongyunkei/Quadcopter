/*
 * MagneticField.h
 *
 *  Created on: 2015¦~9¤ë22¤é
 *      Author: wongy
 */

#ifndef MAGNETICFIELD_H_
#define MAGNETICFIELD_H_

#include <Kalman.h>
#include <MovingWindowAverageFilter.h>
#include <Vector.h>

namespace Sensors{

	class MagneticField{

		public:

			#define GRAVITY		9.80665f

			MagneticField();
			static MagneticField* getInstance();
			void Update();
			float getMagneticField(int);
			void setMagneticField(int, float);
			float getRawMagneticField(int);
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
			MovingWindowAverageFilter* MagneticFieldMovingAverage[3];
	};
};

using namespace Sensors;



#endif /* MAGNETICFIELD_H_ */
