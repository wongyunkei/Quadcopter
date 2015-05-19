/*
 * Kalman.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Kalman.h>
#include <stdio.h>
#include <MathTools.h>

Kalman::Kalman(float Q, float* R, float X, float P) : _Q(0), correctX(0), predictX(0), correctP(0), predictP(0), isOneDim(false){
	_Q = Q;
	_R[0] = R[0];
	_R[1] = R[1];
	_K[0] = 0;
	_K[1] = 0;
	correctX = X;
	predictX = 0;
	correctP = P;
	predictP = 0;
	_yk[0] = 0;
	_yk[1] = 0;
	_Sk[0][0] = 0;
	_Sk[0][1] = 0;
	_Sk[1][0] = 0;
	_Sk[1][1] = 0;
	isOneDim = _R[1] < 0 ? true : false;
}

void Kalman::Filtering(float* output, float data1, float data2){

	StatePredict();
	CovariancePredict();
	MeasurementResidual(data1, data2);
	MeasurementResidualCovariance();
	Gain();
	StateUpdate();
	CovarianceUpdate();
	*output = correctX;
}

void Kalman::MeasurementResidual(float Z1, float Z2){
	_yk[0] = Z1 - predictX;
	if(!isOneDim){
		_yk[1] = Z2 - predictX;
	}
}
void Kalman::MeasurementResidualCovariance(){
	_Sk[0][0] = predictP + _R[0];
	if(!isOneDim){
		_Sk[0][1] = _Sk[1][0] = predictP;
		_Sk[1][1] = predictP + _R[1];
	}
}
void Kalman::StatePredict(){
	predictX = correctX;
}
void Kalman::CovariancePredict(){
	predictP = correctP + _Q;
}
void Kalman::StateUpdate(){

	if(!isOneDim){
		correctX = predictX + _K[0] * _yk[0] + _K[1] * _yk[1];
	}
	else{
		correctX = predictX + _K[0] * _yk[0];
	}
}
void Kalman::CovarianceUpdate(){

	if(!isOneDim){
		correctP = (1 - _K[0] + _K[1]) * predictP;
	}
	else{
		correctP = (1 - _K[0]) * predictP;
	}
}
void Kalman::Gain(){
	if(!isOneDim){

		float inv_Sk[2][2];
		float inv_det = MathTools::TrimResolution(_Sk[0][0] *_Sk[1][1] - _Sk[0][1] *_Sk[1][0]);
		inv_Sk[0][0] = _Sk[1][1] / inv_det;
		inv_Sk[0][1] = _Sk[0][1] / -inv_det;
		inv_Sk[1][0] = _Sk[1][0] / -inv_det;
		inv_Sk[1][1] = _Sk[0][0] / inv_det;
		_K[0] = predictP * inv_Sk[0][0] + predictP * inv_Sk[1][0];
		_K[1] = predictP * inv_Sk[0][1] + predictP * inv_Sk[1][1];
	}
	else{
		float inv_Sk = 1.0 / _Sk[0][0];
		_K[0] = predictP * inv_Sk;
	}
}

void Kalman::Clear(float x){
	predictX = correctX = x;
	predictP = correctP = 1.0;
}

float Kalman::getQ(){
	return _Q;
}

void Kalman::setQ(float q){
	_Q = q;
}

float Kalman::getR1(){
	return _R[0];
}

void Kalman::setR1(float r1){
	_R[0] = r1;
}

float Kalman::getR2(){
	return _R[1];
}

void Kalman::setR2(float r2){
	_R[1] = r2;
}
