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

			Kalman(double, double*, double, double);
			void Filtering(double*, double, double);
			void Clear(double x);
			double getQ();
			void setQ(double q);
			double getR1();
			void setR1(double r1);
			double getR2();
			void setR2(double r2);

		private:

			double _Q, _R[2],_X, _x,_P, _p, _K[2], _yk[2], _Sk[2][2];
			bool isOneDim;
			void MeasurementResidual(double, double);
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
