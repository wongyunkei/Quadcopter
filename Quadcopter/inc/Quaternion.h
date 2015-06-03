/*
 * Quaternion.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <Kalman.h>
#include <UKF.h>
#include <Pid.h>

namespace Math{

	class Quaternion{

		public:
			Quaternion(float);
			static Quaternion* getInstance();
			void Update();
			float getEuler(int);
			void setEuler(int, float);
			float* getQuaternion();
			void getQuaternionConjugate(float*,float*);
			void resetQuaternion();
			Matrix3f getRotationMatrix();
			Matrix3f getPrevRotationMatrix();
			Matrix3f getDeltaRotationMatrix();
			float getInitAngles(int index);
			Kalman* getKalman(int index);
			float getTheater();
			Vector3f temp;


		private:

			float Interval;
			float _Quaternion[4];
			Kalman* _EulerKalman[3];
			UKF* _EulerUKF;
			float _Euler[3];
			float InitAngles[2];
			float PreAcc[3];
			Pid* DriftCorrectionPid[3];
			Matrix3f prevR;
			Matrix3f deltaR;
			void Normalization(float*, float*);
			void QuaternionMultiplication(float*, float*,  float*);
			void EulerToQuaternion(float*, float*);
			void QuaternionToEuler(float*, float*);
			Matrix3f QuaternionToMatrix(float* quaternion);
	};
};

using namespace Math;

#endif /* QUATERNION_H_ */
