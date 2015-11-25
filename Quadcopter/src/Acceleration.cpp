/*
 * Acceleration.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Acceleration.h>
#include <Leds.h>
#include <MathTools.h>
#include <Math.h>
#include <stdio.h>
#include <Quaternion.h>
#include <MPU6050.h>

Acceleration* _mAcceleration[6];

Acceleration::Acceleration(int index) : DevIndex(index), isValided(false){
	accMovingAverage[0] = new MovingWindowAverageFilter(100);
	accMovingAverage[1] = new MovingWindowAverageFilter(100);
	accMovingAverage[2] = new MovingWindowAverageFilter(100);
	Pos[0] = 0.0f;
	Pos[1] = 0.0f;
	Pos[2] = 0.0f;
	Vel[0] = 0.0f;
	Vel[1] = 0.0f;
	Vel[2] = 0.0f;
	_mAcceleration[index] = this;
	AccKalman[0] = new Kalman(MPU6050::getInstance(0)->getRawAcc(0), 0.000001f, 0.001f, 0, true);
	AccKalman[1] = new Kalman(MPU6050::getInstance(0)->getRawAcc(1), 0.000001f, 0.001f, 0, true);
	AccKalman[2] = new Kalman(MPU6050::getInstance(0)->getRawAcc(2), 0.000001f, 0.001f, 0, true);
}

Acceleration* Acceleration::getInstance(int index){
	return _mAcceleration[index];
}

MovingWindowAverageFilter* Acceleration::getMovingAverageFilter(int channel){
	return accMovingAverage[channel];
}

void Acceleration::Update(){

	for(int i = 0; i < 3; i++){
		float temp = MPU6050::getInstance(DevIndex)->getRawAcc(i);
		if(MPU6050::getInstance(DevIndex)->getIsValided()){
			if(temp == temp){
				Acc[i] = temp;
				accMovingAverage[i]->Update(Acc[i]);
				AccKalman[i]->Filtering(Acc[i], 0.0f);
				Acc[i] = AccKalman[i]->getCorrectedData();
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

float Acceleration::getAcc(int channel){
	return Acc[channel];
}

float Acceleration::getVel(int channel){
	return Vel[channel];
}

float Acceleration::getPos(int channel){
	return Pos[channel];
}
float Acceleration::getRawAcc(int channel){
	return RawAcc[channel];
}

void Acceleration::setAcc(int channel, float value){
	Acc[channel] = value;
}

float Acceleration::getAngle(int channel){
	if(channel == 0){
		return atan2(Acc[1], MathTools::Sqrt(Acc[0] * Acc[0] + Acc[2] * Acc[2]));
	}
	else{
		return atan2(-Acc[0], MathTools::Sqrt(Acc[1] * Acc[1] + Acc[2] * Acc[2]));
	}
}

float Acceleration::getFilteredAngle(int channel){
	if(channel == 0){
		return atan2(accMovingAverage[1]->getAverage(), MathTools::Sqrt(accMovingAverage[0]->getAverage() * accMovingAverage[0]->getAverage() + accMovingAverage[2]->getAverage() * accMovingAverage[2]->getAverage()));
	}
	else{
		return atan2(-accMovingAverage[0]->getAverage(), MathTools::Sqrt(accMovingAverage[1]->getAverage() * accMovingAverage[1]->getAverage() + accMovingAverage[2]->getAverage() * accMovingAverage[2]->getAverage()));
	}
}
