/*
 * Acceleration.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Acceleration.h>

using namespace Inertia;

float Acceleration::Gravity = 9.80665f;

Acceleration::Acceleration(Sensors::MPU6050* mMPU6050) : _mMPU6050(mMPU6050), isValided(false){
}

void Acceleration::Update(){
	if(_mMPU6050->getIsValided()){
		Acc = _mMPU6050->getRawAcc();
		isValided = true;
	}
	else{
		isValided = false;
	}
}

bool Acceleration::getIsValided(){
	return isValided;
}

Vector3f Acceleration::getAcc(){
	return Acc;
}

void Acceleration::setAcc(Vector3f value){
	Acc = value;
}

Vector3f Acceleration::getAngle(){
	Vector3f angle;
	angle[0] = asin(MathTools::Trim(-1, Acc[1] / Math::MathTools::Sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1] + Acc[2] * Acc[2]), 1));
	angle[1] = asin(MathTools::Trim(-1, -Acc[0] / Math::MathTools::Sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1] + Acc[2] * Acc[2]), 1));
	angle[2] = 0.0f;
	return angle;
}
