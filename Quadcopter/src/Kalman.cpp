/*
 * Kalman.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Kalman.h>
#include <stdio.h>

Kalman::Kalman(double Q, double* R, double X, double P) : _Q(0), _X(0), _x(0), _P(0), _p(0), isOneDim(false){
	_Q = Q;
	_R[0] = R[0];
	_R[1] = R[1];
	_K[0] = 0;
	_K[1] = 0;
	_X = X;
	_x = 0;
	_P = P;
	_p = 0;
	_yk[0] = 0;
	_yk[1] = 0;
	_Sk[0][0] = 0;
	_Sk[0][1] = 0;
	_Sk[1][0] = 0;
	_Sk[1][1] = 0;
	isOneDim = _R[1] < 0 ? true : false;
}

void Kalman::Filtering(double* output, double data1, double data2){

	StatePredict();
	CovariancePredict();
	MeasurementResidual(data1, data2);
	MeasurementResidualCovariance();
	Gain();
	StateUpdate();
	CovarianceUpdate();
	*output = _X;
}

void Kalman::MeasurementResidual(double Z1, double Z2){
	_yk[0] = Z1 - _x;
	if(!isOneDim){
		_yk[1] = Z2 - _x;
	}
}
void Kalman::MeasurementResidualCovariance(){
	_Sk[0][0] = _p + _R[0];
	if(!isOneDim){
		_Sk[0][1] = _Sk[1][0] = _p;
		_Sk[1][1] = _p + _R[1];
	}
}
void Kalman::StatePredict(){
	_x = _X;
}
void Kalman::CovariancePredict(){
	_p = _P + _Q;
}
void Kalman::StateUpdate(){

	if(!isOneDim){
		_X = _x + _K[0] * _yk[0] + _K[1] * _yk[1];
	}
	else{
		_X = _x + _K[0] * _yk[0];
	}
}
void Kalman::CovarianceUpdate(){

	if(!isOneDim){
		_P = (1 - _K[0] + _K[1]) * _p;
	}
	else{
		_P = (1 - _K[0]) * _p;
	}
}
void Kalman::Gain(){
	if(!isOneDim){

		double inv_Sk[2][2];
		double inv_det = _Sk[0][0] *_Sk[1][1] - _Sk[0][1] *_Sk[1][0];
		inv_Sk[0][0] = _Sk[1][1] / inv_det;
		inv_Sk[0][1] = _Sk[0][1] / -inv_det;
		inv_Sk[1][0] = _Sk[1][0] / -inv_det;
		inv_Sk[1][1] = _Sk[0][0] / inv_det;
		_K[0] = _p * inv_Sk[0][0] + _p * inv_Sk[1][0];
		_K[1] = _p * inv_Sk[0][1] + _p * inv_Sk[1][1];
	}
	else{
		double inv_Sk = 1.0 / _Sk[0][0];
		_K[0] = _p * inv_Sk;
	}
}

void Kalman::Clear(double x){
	_x = _X = x;
	_p = _P = 1.0;
}

double Kalman::getQ(){
	return _Q;
}

void Kalman::setQ(double q){
	_Q = q;
}

double Kalman::getR1(){
	return _R[0];
}

void Kalman::setR1(double r1){
	_R[0] = r1;
}

double Kalman::getR2(){
	return _R[1];
}

void Kalman::setR2(double r2){
	_R[1] = r2;
}
