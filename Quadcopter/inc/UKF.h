/*
 * UKF.h
 *
 *  Created on: 2015¦~4¤ë17¤é
 *      Author: YunKei
 */

#ifndef UKF_H_
#define UKF_H_

//#include <Matrix.h>
#include <Eigen/Eigen>
using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace Math
{
    class UKF
    {
    	public:
    		UKF(Vector3d x, Matrix3d p, Matrix3d q, Matrix3d r, double k = 1);
    		static UKF* getInstance();
    		void getSigmaPoints(Vector3d x[7]);
    		Vector3d Filtering(Vector3d x[7], Vector3d z[7], Vector3d Z);
    		static void printMat(char* ch, int index,  Matrix3d x);
    		static void printVect(char* ch, int index,  Vector3d x);
    	private:

    		void CalcSigmaPoints();
    		void CalcEstimation(Vector3d x[7], Vector3d z[7]);
    		void CalcMeasurement(Vector3d x[7], Vector3d z[7], Vector3d Z);
    		Vector3d predictX;
    		Vector3d correctX;
    		Matrix3d predictP;
    		Matrix3d correctP;
    		Vector3d predictZ;
    		Matrix3d Q;
    		Matrix3d R;
    		Vector3d S[7];
    		double W[7];
    		Matrix3d KalmanGain;
        	double K;
    };
};

using namespace Math;

#endif /* UKF_H_ */
