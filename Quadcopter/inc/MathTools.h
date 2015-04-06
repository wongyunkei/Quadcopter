/*
 * MathTools.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef MATHTOOLS_H_
#define MATHTOOLS_H_

namespace Math{

	class MathTools{

		public:

			#define PI 3.14159f
			#define DEGREE_PER_RADIAN 57.2958f
			#define RADIAN_PER_DEGREE 0.01745f
			static float Sqrt(float);
			static double Sign(double);
			static double DegreeToRadian(double);
			static double RadianToDegree(double);
			static double Trim(double, double, double);
			static double CutOff(double, double, double);
			static double QuadRoot(double);
			static double OctRoot(double);

		private:


	};
};

using namespace Math;

#endif /* MATHTOOLS_H_ */
