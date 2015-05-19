/*
 * SE3.h
 *
 *  Created on: 2015�~5��14��
 *      Author: YunKei
 */

#ifndef SE3_H_
#define SE3_H_

#include <UKF.h>
#include <Kalman.h>
#include <PX4FLOW.h>
#include <stdio.h>
#include <Quaternion.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::Matrix4f;
using Eigen::Matrix3f;

namespace Math{
	class SE3{
		public:
			SE3();
			static SE3* getInstance();
			void Update();
			Vector3f getPos();
			static void printMat(char* ch, int index,  Matrix4f x);
			static void printVect(char* ch, int index, Vector4f x);
			Matrix4f getSE3();
			void reset();
		private:
			UKF* SE3UKF;
			Kalman* SE3Kalman[3];
			Vector3f prevTranslation;
			Matrix4f _SE3;
			Vector4f _pos;
			Vector3f pos;
	};
};

using namespace Math;

#endif /* SE3_H_ */
