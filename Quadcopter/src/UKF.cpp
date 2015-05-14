/*
 * UKF.cpp
 *
 *  Created on: 2015¦~4¤ë17¤é
 *      Author: YunKei
 */

#include <UKF.h>
#include <math.h>
#include <MathAlgorithm.h>

extern uint32_t heacorrectPloc;

UKF* _mUKF;

void UKF::printMat(char* ch, int index,  Matrix3d x){
	printf("\n%s:%d\n", ch, index);
	printf("%g  %g  %g\n", x(0,0), x(0,1), x(0,2));
	printf("%g  %g  %g\n", x(1,0), x(1,1), x(1,2));
	printf("%g  %g  %g\n", x(2,0), x(2,1), x(2,2));

}

void UKF::printVect(char* ch, int index,  Vector3d x){
	printf("\n%s:%d\n", ch, index);
	printf("%g  %g  %g\n", x(0), x(1), x(2));

}

UKF::UKF(Vector3d x, Matrix3d p, Matrix3d q, Matrix3d r, double k){
	_mUKF = this;
	K = k;
	predictX.setZero();
	correctX.setZero();
	predictP.setZero();
	correctP.setZero();
	predictZ.setZero();
	KalmanGain.setZero();
	Q.setZero();
	R.setZero();
	correctX = x;
	correctP = p;
	Q = q;
	R = r;
	for (int i = 0; i < 3; i++){
		S[i].setZero();
	}
	S[3] = correctX;
	double sumW = W[3] = K / (3 + K);
	for (int i = 0; i < 3; i++)
	{
		W[i] = 1 / (2 * (3 + K));
		W[4 + i] = 1 / (2 * (3 + K));
		sumW += W[i];
		sumW += W[4 + i];
	}
	for (int i = 0; i < 7; i++)
	{
		W[i] /= sumW;
	}
	CalcSigmaPoints();
}

UKF* UKF::getInstance(){
	return _mUKF;
}

void UKF::CalcSigmaPoints()
{
	S[3] = correctX;
	Matrix3d P;
	Matrix3d D;
	Matrix3d Q;
	Matrix3d invQ;
	P = correctP;
	MathAlgorithm::EigenDecomposition(P, D, Q);
	for (int j = 0; j < 3; j++)
	{
		D(j,j) = sqrt(D(j,j));
	}
	bool isInvertable = false;
	Q.computeInverseWithCheck(invQ, isInvertable);
	correctP = Q * D * invQ;
	correctP *= sqrt(3 + K);
	for(int i = 0; i < 3; i++){
		Vector3d v;
		v.setZero();
		for(int j = 0; j < 3; j++){
			v(j,0)	= correctP(j,i);
		}
		S[i] = correctX + v;
		S[4 + i] = correctX - v;
	}
}

void UKF::CalcEstimation(Vector3d x[7], Vector3d z[7])
{
    predictX.setZero();
    predictP.setZero();
    predictZ.setZero();
    for (int i = 0; i < 7; i++)
    {
        predictX += x[i]*W[i];
        predictZ += z[i]*W[i];
    }
    for(int i = 0; i < 3; i++){
		if(predictX(i) < 0.000000000001){
			predictX(i) = 0;
		}
		if(predictZ(i) < 0.000000000001){
			predictZ(i) = 0;
		}
	}
    for (int i = 0; i < 7; i++)
    {
        predictP += (S[i] - predictX) * (S[i] - predictX).transpose()*W[i];
    }
    predictP += Q;
}

void UKF::CalcMeasurement(Vector3d x[7], Vector3d z[7], Vector3d Z)
{
    Matrix3d Pzz;
    Matrix3d Pxz;
    Matrix3d invPzz;
    Pzz.setZero();
    Pxz.setZero();
    for (int i = 0; i < 7; i++)
    {
        Pzz += (z[i] - predictZ) * (z[i] - predictZ).transpose()*W[i];
        Pxz += (x[i] - predictX) * (z[i] - predictZ).transpose()*W[i];
    }
    Pzz += R;
    bool isInvertable = false;
    Pzz.computeInverseWithCheck(invPzz, isInvertable);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
        	if(Pzz(i,j) < 0.000000000001){
        		Pzz(i,j) = 0;
        	}
        	if(Pxz(i,j) < 0.000000000001){
        		Pxz(i,j) = 0;
        	}
        	if(invPzz(i,j) < 0.000000000001){
        		invPzz(i,j) = 0;
        	}
        }
    }
    KalmanGain = Pxz * invPzz;
    correctX = predictX + (KalmanGain * (Z - predictZ));
    correctP = predictP - (KalmanGain * Pzz * KalmanGain.transpose());
    for(int i = 0; i < 3; i++){
    	if(correctX(i) < 0.000000000001){
			correctX(i) = 0;
		}
        for(int j = 0; j < 3; j++){
        	if(correctP(i,j) < 0.000000000001){
        		correctP(i,j) = 0;
        	}
        }
    }
//	printMat("Pzz",0, Pzz);
//	printMat("invPzz",0, invPzz);
//	printMat("Pxz",0, Pxz);
}

void UKF::getSigmaPoints(Vector3d x[7]){
	for (int i = 0; i < 7; i++)
	{
		x[i] = S[i];
	}
}

Vector3d UKF::Filtering(Vector3d x[7], Vector3d z[7], Vector3d Z)
{
//	for(int i = 0; i < 7; i++){
//		printVect("x",i, x[i]);
//		printVect("z",i, z[i]);
//	}
	CalcEstimation(x, z);
//	printVect("predictX",0, predictX);
//	printMat("predictP",0, predictP);
//	printVect("predictZ",0, predictZ);
	CalcMeasurement(x, z, Z);
//	printVect("correctX",0, correctX);
//	printMat("correctP",0, correctP);
//	printMat("KalmanGain",0, KalmanGain);
	CalcSigmaPoints();
//	for(int i = 0; i < 7; i++){
//		printVect("S",i, S[i]);
//	}
	return correctX;
}
