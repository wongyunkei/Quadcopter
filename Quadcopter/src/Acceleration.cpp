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

Acceleration* _mAcceleration;

Acceleration::Acceleration(){
	accMovingAverage[0] = new MovingWindowAverageFilter(50);
	accMovingAverage[1] = new MovingWindowAverageFilter(50);
	accMovingAverage[2] = new MovingWindowAverageFilter(50);
	_mAcceleration = this;

//	double R[3][2] = {{0.001, -1},
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
		Acc[i] = MPU6050::getInstance()->getRawAcc(i);
//		AccKalman[i]->Filtering(&Acc[i], MPU6050::getInstance()->getRawAcc(i), 0.0);
		accMovingAverage[i]->Update(Acc[i]);
	}

}

double Acceleration::getAcc(int index){
	return Acc[index];
}

double Acceleration::getRawAcc(int index){
	return RawAcc[index];
}

void Acceleration::setAcc(int index, double value){
	Acc[index] = value;
}

double Acceleration::getAngle(int index){
	if(index == 0){
		return atan2(Acc[1], MathTools::Sqrt(Acc[0] * Acc[0] + Acc[2] * Acc[2]));
	}
	else{
		return atan2(-Acc[0], MathTools::Sqrt(Acc[1] * Acc[1] + Acc[2] * Acc[2]));
	}
}

double Acceleration::getFilteredAngle(int index){
	if(index == 0){
		return atan2(accMovingAverage[1]->getAverage(), MathTools::Sqrt(accMovingAverage[0]->getAverage() * accMovingAverage[0]->getAverage() + accMovingAverage[2]->getAverage() * accMovingAverage[2]->getAverage()));
	}
	else{
		return atan2(-accMovingAverage[0]->getAverage(), MathTools::Sqrt(accMovingAverage[1]->getAverage() * accMovingAverage[1]->getAverage() + accMovingAverage[2]->getAverage() * accMovingAverage[2]->getAverage()));
	}
}
