/*
 * Quaternion.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <AdaptiveKalman.h>
#include <UKF.h>
#include <Pid.h>

namespace Math{

	class Quaternion{

		public:
			Quaternion(int index, float interval);
			static Quaternion* getInstance(int index);
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
			float temp1;
			float temp2;
			float temp3;


		private:

			float Interval;
			int DevIndex;
			float _Quaternion[4];
//			Kalman* _EulerKalman[3];
			AdaptiveKalman* _EulerKalman[3];
			UKF* _EulerUKF;
			float _Euler[3];
			float InitAngles[2];
			float Acc[3];
			float Angle[2];
			float prevAngle[2];
			bool prevValid;
			bool Valid;
			Pid* DriftCorrectionPid[3];
			Matrix3f prevR;
			Matrix3f deltaR;
			void Normalization(float*, float*);
			void QuaternionMultiplication(float*, float*,  float*);
			void EulerToQuaternion(float*, float*);
			void QuaternionToEuler(float*, float*);
			Matrix3f QuaternionToMatrix(float* quaternion);
			Matrix3f EulerToMatrix(float* euler);
			void MatrixToEuler(Matrix3f m, float* euler);
			void MatrixToQuaternion(Matrix3f m, float* quaternion);
			void MatrixToFixedAngles(Matrix3f m, float* fixedAngles);
			void FixedAnglesToQuaternion(float* fixedAngles, float* quaternion);
			void resetQuaternionTask();
	};
};

using namespace Math;

#endif /* QUATERNION_H_ */
