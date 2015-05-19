/*
 * Kalman.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef KALMAN_H_
#define KALMAN_H_

namespace Math{

	class Kalman{

		public:

			Kalman(float, float*, float, float);
			void Filtering(float*, float, float);
			void Clear(float x);
			float getQ();
			void setQ(float q);
			float getR1();
			void setR1(float r1);
			float getR2();
			void setR2(float r2);

		private:

			float _Q;
			float _R[2];
			float correctX;
			float  predictX;
			float predictP;
			float correctP;
			float _K[2];
			float _yk[2];
			float _Sk[2][2];
			bool isOneDim;
			void MeasurementResidual(float, float);
			void MeasurementResidualCovariance();
			void StatePredict();
			void CovariancePredict();
			void StateUpdate();
			void CovarianceUpdate();
			void Gain();
	};
};

using namespace Math;

#endif /* KALMAN_H_ */
