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
	mAccMovingWindowAverageFilter[0] = new MovingWindowAverageFilter(20);
	mAccMovingWindowAverageFilter[1] = new MovingWindowAverageFilter(20);
	mAccMovingWindowAverageFilter[2] = new MovingWindowAverageFilter(20);
	Update();
}

void Acceleration::Update(){
	if(_mMPU6050->getIsValided()){
		Acc = _mMPU6050->getRawAcc();
		for(int i = 0; i < 3; i++){
			mAccMovingWindowAverageFilter[i]->Update(Acc[i]);
		}
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

Vector3f Acceleration::getFilteredAcc(){
	Vector3f acc;
	acc << mAccMovingWindowAverageFilter[0]->getAverage(),
		   mAccMovingWindowAverageFilter[1]->getAverage(),
		   mAccMovingWindowAverageFilter[2]->getAverage();
	return acc;
}

Vector3f Acceleration::getFilteredAngle(){
	Vector3f acc = getFilteredAcc();
	Vector3f angle;
	angle[0] = atan2(acc[1], acc[2]);
	angle[1] = atan2(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));

//	angle[0] = asin(MathTools::Trim(-1, acc[1] / sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]), 1));
//	angle[1] = asin(MathTools::Trim(-1, -acc[0] / sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]), 1));
//	if(acc[2] < 0 && acc[1] > 0){
//		angle[0] = MathTools::PI - angle[0];
//	}
//	else if(acc[2] < 0 && acc[1] < 0){
//		angle[0] = -MathTools::PI - angle[0];
//	}
//
//	if(acc[2] < 0 && acc[0] < 0){
//		angle[1] = MathTools::PI - angle[1];
//	}
//	else if(acc[2] < 0 && acc[0] > 0){
//		angle[1] = -MathTools::PI - angle[1];
//	}
	angle[2] = 0.0f;
	return angle;
}

void Acceleration::setAcc(Vector3f value){
	Acc = value;
}

Vector3f Acceleration::getAngle(){
	Vector3f angle;

	angle[0] = atan2(Acc[1], Acc[2]);
	angle[1] = atan2(-Acc[0], sqrtf(Acc[1] * Acc[1] + Acc[2] * Acc[2]));
//	angle[0] = asin(MathTools::Trim(-1, Acc[1] / Math::MathTools::Sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1] + Acc[2] * Acc[2]), 1));
//	angle[1] = asin(MathTools::Trim(-1, -Acc[0] / Math::MathTools::Sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1] + Acc[2] * Acc[2]), 1));

//	if(Acc[2] < 0 && Acc[1] > 0){
//		angle[0] = MathTools::PI - angle[0];
//	}
//	else if(Acc[2] < 0 && Acc[1] < 0){
//		angle[0] = -MathTools::PI - angle[0];
//	}
//
//	if(Acc[2] < 0 && Acc[0] < 0){
//		angle[1] = MathTools::PI - angle[1];
//	}
//	else if(Acc[2] < 0 && Acc[0] > 0){
//		angle[1] = -MathTools::PI - angle[1];
//	}
	angle[2] = 0.0f;
	return angle;
}
