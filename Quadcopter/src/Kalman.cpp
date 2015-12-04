/*
 * Kalman.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Kalman.h>
#include <stdio.h>
#include <MathTools.h>

Kalman::Kalman(float x, float q, float r1, bool isOneDim, float r2) : _Q(q), correctX(x), predictX(0), correctP(0), predictP(0), IsOneDim(isOneDim){
	_R[0] = r1;
	_R[1] = r2;
	_K[0] = 0;
	_K[1] = 0;
	_yk[0] = 0;
	_yk[1] = 0;
	_Sk[0][0] = 0;
	_Sk[0][1] = 0;
	_Sk[1][0] = 0;
	_Sk[1][1] = 0;
}

void Kalman::setCorrectedData(float data){
	correctX = data;
}
float Kalman::getCorrectedData(){
	return correctX;
}

bool Kalman::getIsOneDim(){
	return IsOneDim;
}

void Kalman::setIsOneDim(bool value){
	IsOneDim = value;
}

void Kalman::Filtering(float data1, float data2){

	StatePredict();
	CovariancePredict();
	MeasurementResidual(data1, data2);
	MeasurementResidualCovariance();
	Gain();
	StateUpdate();
	CovarianceUpdate();
}

void Kalman::MeasurementResidual(float Z1, float Z2){
	_yk[0] = Z1 - predictX;
	if(!IsOneDim){
		_yk[1] = Z2 - predictX;
	}
}
void Kalman::MeasurementResidualCovariance(){
	_Sk[0][0] = predictP + _R[0];
	if(!IsOneDim){
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

	if(!IsOneDim){
		correctX = predictX + _K[0] * _yk[0] + _K[1] * _yk[1];
	}
	else{
		correctX = predictX + _K[0] * _yk[0];
	}
}
void Kalman::CovarianceUpdate(){

	if(!IsOneDim){
		correctP = (1 - _K[0] + _K[1]) * predictP;
	}
	else{
		correctP = (1 - _K[0]) * predictP;
	}
}
void Kalman::Gain(){
	if(!IsOneDim){

		double inv_Sk[2][2];
		double inv_det = _Sk[0][0] *_Sk[1][1] - _Sk[0][1] *_Sk[1][0];
		inv_Sk[0][0] = _Sk[1][1] / inv_det;
		inv_Sk[0][1] = _Sk[0][1] / -inv_det;
		inv_Sk[1][0] = _Sk[1][0] / -inv_det;
		inv_Sk[1][1] = _Sk[0][0] / inv_det;
		_K[0] = predictP * inv_Sk[0][0] + predictP * inv_Sk[1][0];
		_K[1] = predictP * inv_Sk[0][1] + predictP * inv_Sk[1][1];
	}
	else{
		double inv_Sk = 1.0 / _Sk[0][0];
		_K[0] = predictP * inv_Sk;
	}
}

void Kalman::Clear(float x){
	predictX = correctX = x;
	predictP = correctP = 0.0;
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
