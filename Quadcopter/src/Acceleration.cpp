/*
 * Acceleration.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Acceleration.h>
#include <MPU6050.h>
#include <MathTools.h>
#include <Math.h>
#include <Leds.h>
#include <stdio.h>
#include <Quaternion.h>

Acceleration* _mAcceleration;

Acceleration::Acceleration() : isValided(false){
	accMovingAverage[0] = new MovingWindowAverageFilter(50);
	accMovingAverage[1] = new MovingWindowAverageFilter(50);
	accMovingAverage[2] = new MovingWindowAverageFilter(50);
	Pos[0] = 0.0f;
	Pos[1] = 0.0f;
	Pos[2] = 0.0f;
	Vel[0] = 0.0f;
	Vel[1] = 0.0f;
	Vel[2] = 0.0f;
	_mAcceleration = this;

//	float R[3][2] = {{0.001, -1},
//					{0.001, -1},
//					{0.001, -1}};
//	AccKalman[0] = new Kalman(0.000001, R[0], MPU6050::getInstance()->getRawAcc(0), 1);
//	AccKalman[1] = new Kalman(0.000001, R[1], MPU6050::getInstance()->getRawAcc(1), 1);
//	AccKalman[2] = new Kalman(0.000001, R[2], MPU6050::getInstance()->getRawAcc(2), 1);
}

Acceleration* Acceleration::getInstance(){
	return _mAcceleration;
}

MovingWindowAverageFilter* Acceleration::getMovingAverageFilter(int index){
	return accMovingAverage[index];
}

void Acceleration::Update(){

	for(int i = 0; i < 3; i++){
		float temp = MPU6050::getInstance()->getRawAcc(i);
		if(MPU6050::getInstance()->getIsValided()){
			if(temp == temp){
				Acc[i] = temp;
				accMovingAverage[i]->Update(Acc[i]);
				isValided = true;
			}
			else{
				isValided = false;
			}
		}
		else{
			isValided = false;
		}
//		AccKalman[i]->Filtering(&Acc[i], MPU6050::getInstance()->getRawAcc(i), 0.0);

	}
	if(isValided){

//		float g[3] = {-GRAVITY*sinf(getFilteredAngle(0)),-GRAVITY*sinf(getFilteredAngle(1)),-GRAVITY*cosf(getFilteredAngle(0))*cosf(getFilteredAngle(1))};
//		float a[3] = {accMovingAverage[0]->getAverage(), accMovingAverage[1]->getAverage(),accMovingAverage[2]->getAverage()};
//		Vector::Add(a, a, g);
//		Vector::Scale(a,a,0.002f);
//		for(int i = 0; i < 3; i++){
//			Vel[i] += a[i];
//			Pos[i] += Vel[i] * 0.002f;
//		}
	}

}

bool Acceleration::getIsValided(){
	return isValided;
}

float Acceleration::getAcc(int index){
	return Acc[index];
}

float Acceleration::getVel(int index){
	return Vel[index];
}

float Acceleration::getPos(int index){
	return Pos[index];
}
float Acceleration::getRawAcc(int index){
	return RawAcc[index];
}

void Acceleration::setAcc(int index, float value){
	Acc[index] = value;
}

float Acceleration::getAngle(int index){
	if(index == 0){
		return atan2(Acc[1], MathTools::Sqrt(Acc[0] * Acc[0] + Acc[2] * Acc[2]));
	}
	else{
		return atan2(-Acc[0], MathTools::Sqrt(Acc[1] * Acc[1] + Acc[2] * Acc[2]));
	}
}

float Acceleration::getFilteredAngle(int index){
	if(index == 0){
		return atan2(-accMovingAverage[1]->getAverage(), MathTools::Sqrt(accMovingAverage[0]->getAverage() * accMovingAverage[0]->getAverage() + accMovingAverage[2]->getAverage() * accMovingAverage[2]->getAverage()));
	}
	else{
		return atan2(accMovingAverage[0]->getAverage(), MathTools::Sqrt(accMovingAverage[1]->getAverage() * accMovingAverage[1]->getAverage() + accMovingAverage[2]->getAverage() * accMovingAverage[2]->getAverage()));
	}
}
