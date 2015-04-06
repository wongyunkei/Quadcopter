/*
 * Quaternion.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <Kalman.h>
#include <Pid.h>

namespace Math{

	class Quaternion{

		public:

			Quaternion(double);
			static Quaternion* getInstance();
			void Update();
			double getEuler(int);
			void setEuler(int, double);
			double* getQuaternion();
			void getQuaternionConjugate(double*,double*);
			void resetQuaternion();
			void QuaternionToMatrix(double*, double[3][3]);
			double getInitAngles(int index);
			Kalman* getKalman(int index);
			double temp1[3];
			double temp2[3];

		private:

			double Interval;
			double _Quaternion[4];
			Kalman* _EulerKalman[3];
			double _Euler[3];
			double InitAngles[2];
			double PreAcc[3];
			Pid* DriftCorrectionPid[3];
			void Normalization(double*, double*);
			void QuaternionMultiplication(double*, double*,  double*);
			void EulerToQuaternion(double*, double*);
			void QuaternionToEuler(double*, double*);
	};
};

using namespace Math;

#endif /* QUATERNION_H_ */
