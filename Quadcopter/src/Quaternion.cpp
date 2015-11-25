/*
 * Quaternion.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Quaternion.h>
#include <MathTools.h>
#include <math.h>
#include <Omega.h>
#include <Acceleration.h>
#include <Vector.h>
#include <Kalman.h>
#include <MPU6050.h>
#include <stdio.h>
#include <Pid.h>
#include <Usart.h>
#include <Task.h>
#include <HMC5883L.h>
#include <Controlling.h>
#include <Delay.h>
#include <AdditionalTools.h>
#include <Leds.h>
#include <Communicating.h>

Quaternion* _mQuaternion[6];

Quaternion::Quaternion(int index, float interval) : DevIndex(index), Interval(interval), Valid(false), prevValid(false){

	_mQuaternion[index] = this;

	InitAngles[0] = _Euler[0] = Acceleration::getInstance(index)->getFilteredAngle(0);
	InitAngles[1] = _Euler[1] = Acceleration::getInstance(index)->getFilteredAngle(1);
	_Euler[2] = 0;
	EulerToQuaternion(_Euler, _Quaternion);
	prevAngle[0] = 0;
	prevAngle[1] = 0;
//	_Quaternion[0] = 1;
//	_Quaternion[1] = 0;
//	_Quaternion[2] = 0;
//	_Quaternion[3] = 0;

//	float R[2] = {0.00075, 0.020472997};
//	_EulerKalman[0] = new Kalman(0.0001, R, _Euler[0], 1.0);
//	R[0] = 0.0021;
//	R[1] = 0.004316973;
//	_EulerKalman[1] = new Kalman(0.0001, R, _Euler[1], 1.0);
//	R[0] = 0.000128;
//	R[1] = 0.001349584;
//	_EulerKalman[2] = new Kalman(0.0001, R, 0, 1.0);
	Matrix3f m = QuaternionToMatrix(_Quaternion);

//	float R[2] = {0.00011f, 0.00001f};
//	_EulerKalman[0] = new Kalman(0.000001f, R, m(2,0), 0.0f);
//	_EulerKalman[1] = new Kalman(0.000001f, R, m(2,1), 0.0f);
//	_EulerKalman[2] = new Kalman(0.000001f, R, m(2,2), 0.0f);
//
	_EulerKalman[0] = new AdaptiveKalman(m(2,0), 1.0f, 0.001f, 0.000001f);
	_EulerKalman[1] = new AdaptiveKalman(m(2,1), 1.0f, 0.001f, 0.000001f);
	_EulerKalman[2] = new AdaptiveKalman(m(2,2), 1.0f, 0.001f, 0.000001f);



//	Vector3d x;
//	Matrix3f p;
//	Matrix3f q;
//	Matrix3f r;
//	x.setZero();
//	p.setIdentity();
//	q.setZero();
//	r.setZero();
//	q(0,0) = q(1,1) = q(2,2) = 0.0001;
//	r(0,0) = 0.5;
//	r(1,1) = 0.5;
//	r(2,2) = 0.5;
//	_EulerUKF = new UKF(x, p, q, r);

	DriftCorrectionPid[0] = new Pid(5,2.5f,0.0f,0,1000,interval);
	DriftCorrectionPid[1] = new Pid(6,2.5f,0.0f,0,1000,interval);
	DriftCorrectionPid[2] = new Pid(7,2.5f,0.0f,0,1000,interval);
	prevR.setZero();
}

Quaternion* Quaternion::getInstance(int index){
	return _mQuaternion[index];
}

Kalman* Quaternion::getKalman(int index){
	return _EulerKalman[index];
}

float Quaternion::getInitAngles(int index){
	return InitAngles[index];
}

float Quaternion::getEuler(int index){
	return _Euler[index];
}

void Quaternion::setEuler(int index, float angle){
	_Euler[index] = angle;
	float quaternion[4];
	EulerToQuaternion(_Euler, quaternion);
	for(int i = 0; i < 4; i++){
		_Quaternion[i] = quaternion[i];
	}
}


void resetUpdate(int index){
	MPU6050::getInstance(index)->Update();
	Omega::getInstance(index)->Update();
	Acceleration::getInstance(index)->Update();
	Quaternion::getInstance(index)->Update();
}

void resetTask(){
	Controlling::getInstant()->setRPYOffset(0, -MathTools::RadianToDegree(Quaternion::getInstance(0)->getEuler(0)));
	Controlling::getInstant()->setRPYOffset(1, -MathTools::RadianToDegree(Quaternion::getInstance(0)->getEuler(1)));
	Controlling::getInstant()->setRPYOffset(2, -MathTools::RadianToDegree(Quaternion::getInstance(0)->getEuler(2)));
}

void Quaternion::resetQuaternionTask(){
//	for(int i = 0; i < 500; i++){
//		MPU6050::getInstance(DevIndex)->Update();
//		Acceleration::getInstance(DevIndex)->Update();
//		Omega::getInstance(DevIndex)->Update();
//		Quaternion::getInstance(DevIndex)->Update();
//		Delay::DelayMS(Interval*1000);
//	}
	Controlling::getInstant()->setRPYOffset(0, -MathTools::RadianToDegree(Quaternion::getInstance(DevIndex)->getEuler(0)));
	Controlling::getInstant()->setRPYOffset(1, -MathTools::RadianToDegree(Quaternion::getInstance(DevIndex)->getEuler(1)));
	Controlling::getInstant()->setRPYOffset(2, -MathTools::RadianToDegree(Quaternion::getInstance(DevIndex)->getEuler(2)));
}

void Quaternion::resetQuaternion(){
	InitAngles[0] = _Euler[0] = Acceleration::getInstance(DevIndex)->getFilteredAngle(0);
	InitAngles[1] = _Euler[1] = Acceleration::getInstance(DevIndex)->getFilteredAngle(1);
	_Euler[2] = 0;
	EulerToQuaternion(_Euler, _Quaternion);
	Matrix3f m = QuaternionToMatrix(_Quaternion);

	for(int i = 0; i < 3; i++){
		_EulerKalman[i]->Clear(m(2,i));
	}
	Task::getInstance()->Attach(500, 499, resetTask, false, 1);
//	resetQuaternionTask();
//	for(int i = 0; i < 3; i++){
//		DriftCorrectionPid[i]->clear();
//	}
}

float* Quaternion::getQuaternion(){
	return _Quaternion;
}

void Quaternion::getQuaternionConjugate(float* conjugate,float* quaternion){
	conjugate[0] = quaternion[0];
	conjugate[1] = -quaternion[1];
	conjugate[2] = -quaternion[2];
	conjugate[3] = -quaternion[3];
}

void Quaternion::Update(){
	if(Omega::getInstance(DevIndex)->getIsValided() == false){
		return;
	}
	if(Acceleration::getInstance(DevIndex)->getIsValided() == false){
		return;
	}

	float DeltaQuaternion[4] = {0,0,0,0};
	DeltaQuaternion[0] = 0;
	for(int i = 0; i < 3; i++){
		DeltaQuaternion[i + 1] = MathTools::DegreeToRadian(Omega::getInstance(DevIndex)->getOmega(i));
	}

	QuaternionMultiplication(DeltaQuaternion, _Quaternion, DeltaQuaternion);

	for(int i = 0; i < 4; i++){
		DeltaQuaternion[i] *= Interval / 2.0f;
		_Quaternion[i] += DeltaQuaternion[i];
	}

	Normalization(_Quaternion, _Quaternion);

	Matrix3f m = QuaternionToMatrix(_Quaternion);
	Vector3f acc;
	for(int i = 0; i < 3; i++){
		acc[i] = Acceleration::getInstance(DevIndex)->getMovingAverageFilter(i)->getAverage();
		acc[i] *= 1.0 / GRAVITY;

	}
	acc[2] = -acc[2];

	float mag = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

	Valid = mag < 1.05f && mag > 0.95f ? true : false;
	for(int i = 0; i < 3; i++){
		acc[i] = MathTools::Trim(-1, acc[i], 1);
	}
	if(Valid){
//	if(false){
		Vector3f v;
		v[0] = m(2,0);
		v[1] = m(2,1);
		v[2] = m(2,2);
		for(int i = 0; i < 3; i++){
			_EulerKalman[i]->Filtering(v[i], acc[i]);
			float t[3] = {_EulerKalman[i]->getQ(), _EulerKalman[i]->getR1(), _EulerKalman[i]->getR2()};
			AdditionalTools::setBuffer(0, t, 3);
			m(2,i) = _EulerKalman[i]->getCorrectedData();
		}

//		Vector3f z = v.cross(acc);
//		for(int i = 0; i < 3; i++){
////			 v[i] += DriftCorrectionPid[i]->pid(0, z[i]);
////			 m(2,i) = v[i];
//		}

		MatrixToQuaternion(m, _Quaternion);
		Normalization(_Quaternion, _Quaternion);
	}
	QuaternionToEuler(_Quaternion, _Euler);
}

void Quaternion::Normalization(float* quaternion, float* quaternionNorm){
	float mag = sqrtf(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);

	for(int i = 0; i < 4; i++){
		quaternionNorm[i] = quaternion[i] / mag;
		quaternionNorm[i] = MathTools::Trim(-1, quaternionNorm[i], 1);
	}
}

void Quaternion::QuaternionMultiplication(float* quaternion, float* quaternion1,  float* quaternion2){
	float temp[4];
	temp[0] = quaternion1[0] * quaternion2[0] - quaternion1[1] * quaternion2[1] - quaternion1[2] * quaternion2[2] - quaternion1[3] * quaternion2[3];
	float v[3];
	Vector::CrossProduct(v, &quaternion1[1], &quaternion2[1]);
	temp[1] = quaternion1[0] * quaternion2[1] + quaternion1[1] * quaternion2[0] + v[0];
	temp[2] = quaternion1[0] * quaternion2[2] + quaternion1[2] * quaternion2[0] + v[1];
	temp[3] = quaternion1[0] * quaternion2[3] + quaternion1[3] * quaternion2[0] + v[2];
	for(int i = 0; i < 4; i++){
		quaternion[i] = temp[i];
	}
}

void Quaternion::EulerToQuaternion(float* euler, float* quaternion){
	float fixedAngles[3];
	MatrixToFixedAngles(EulerToMatrix(euler), fixedAngles);
	FixedAnglesToQuaternion(fixedAngles, quaternion);
}

void Quaternion::FixedAnglesToQuaternion(float* fixedAngles, float* quaternion){
	float CosHalfRoll = cosf(fixedAngles[0] / 2.0f), SinHalfRoll = sinf(fixedAngles[0] / 2.0f);
	float CosHalfPitch = cosf(fixedAngles[1] / 2.0f), SinHalfPitch = sinf(fixedAngles[1] / 2.0f);
	float CosHalfYaw = cosf(fixedAngles[2] / 2.0f), SinHalfYaw = sinf(fixedAngles[2] / 2.0f);
	quaternion[0] = CosHalfRoll * CosHalfPitch * CosHalfYaw - SinHalfRoll * SinHalfPitch * SinHalfYaw;
	quaternion[1] = CosHalfRoll * SinHalfPitch * SinHalfYaw + SinHalfRoll * CosHalfPitch * CosHalfYaw;
	quaternion[2] = CosHalfRoll * SinHalfPitch * CosHalfYaw - SinHalfRoll * CosHalfPitch * SinHalfYaw;
	quaternion[3] = CosHalfRoll * CosHalfPitch * SinHalfYaw + SinHalfRoll * SinHalfPitch * CosHalfYaw;
}

void Quaternion::MatrixToQuaternion(Matrix3f m, float* quaternion){
	float fixedAngles[3];
	MatrixToFixedAngles(m, fixedAngles);
	FixedAnglesToQuaternion(fixedAngles, quaternion);
}

void Quaternion::MatrixToEuler(Matrix3f m, float* euler){
	euler[0] = atan2f((float)m(2,1), (float)m(2,2));
	euler[1] = asinf((float)-m(2,0));
	euler[2] = atan2f((float)m(0,1), (float)m(0,0));
}

void Quaternion::QuaternionToEuler(float* quaternion, float* euler){
//	float r32 = 2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]);
//	float r33 = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
//	float r2_32 = r32 * r32;
//	float r2_33 = r33 * r33;
//	float l = sqrtf(r2_32 + r2_33);
	euler[0] = atan2f(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]));
	euler[1] = asinf(2 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3]));
//	euler[1] = atan2f(2 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3]), l);
	euler[2] = atan2f(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]), 1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]));
}

Matrix3f Quaternion::EulerToMatrix(float* euler){
	Matrix3f m;
	float CosRoll = cosf(euler[0]), SinRoll = sinf(euler[0]);
	float CosPitch = cosf(euler[1]), SinPitch = sinf(euler[1]);
	float CosYaw = cosf(euler[2]), SinYaw = sinf(euler[2]);
	m(0,0) = CosYaw * CosPitch;
	m(0,1) = -SinYaw * CosRoll + CosYaw * SinPitch * SinRoll;
	m(0,2) = SinYaw * SinRoll + CosYaw * SinPitch * CosRoll;
	m(1,0) = SinYaw * CosPitch;
	m(1,1) = CosYaw * CosRoll + SinYaw * SinPitch * SinRoll;
	m(1,2) = -CosYaw * SinRoll + SinYaw * SinPitch * CosRoll;
	m(2,0) = -SinPitch;
	m(2,1) = CosPitch * SinRoll;
	m(2,2) = CosPitch * CosRoll;
	return m;
}

void Quaternion::MatrixToFixedAngles(Matrix3f m, float* fixedAngles){
	fixedAngles[0] = atan2f((float)-m(1,2), (float)m(2,2));
	fixedAngles[1] = asinf((float)m(0,2));
	fixedAngles[2] = atan2f((float)-m(0,1), (float)m(0,0));
}

Matrix3f Quaternion::QuaternionToMatrix(float* quaternion){
	QuaternionToEuler(_Quaternion, _Euler);
	return EulerToMatrix(_Euler);
}

Matrix3f Quaternion::getRotationMatrix(){
	return QuaternionToMatrix(_Quaternion);
}

Matrix3f Quaternion::getPrevRotationMatrix(){
	return prevR;
}

Matrix3f Quaternion::getDeltaRotationMatrix(){
	return deltaR;
}

float Quaternion::getTheater(){
	return acosf((getRotationMatrix().trace() - 1.0) / 2.0) ;
}
