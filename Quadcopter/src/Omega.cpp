/*
 * Omega.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Omega.h>
#include <MPU6050.h>
#include <MathTools.h>

Omega* _mOmega;

Omega::Omega() : isValided(false){
	_mOmega = this;

//	float R[3][2] = {{0.0001, -1},
//					{0.0001, -1},
//					{0.0001, -1}};
//	OmegaKalman[0] = new Kalman(0.000001, R[0], MPU6050::getInstance()->getRawOmega(0), 1);
//	OmegaKalman[1] = new Kalman(0.000001, R[1], MPU6050::getInstance()->getRawOmega(1), 1);
//	OmegaKalman[2] = new Kalman(0.000001, R[2], MPU6050::getInstance()->getRawOmega(2), 1);
}

Omega* Omega::getInstance(){
	return _mOmega;
}

void Omega::Update(){

	for(int i = 0; i < 3; i++){
		float temp = MPU6050::getInstance()->getRawOmega(i);
		if(MPU6050::getInstance()->getIsValided()){
			if(temp == temp){
		//		OmegaKalman[i]->Filtering(&_Omega[i], MPU6050::getInstance()->getRawOmega(i), 0.0);
				_Omega[i] = temp;
				isValided = true;
			}
			else{
				isValided = false;
			}
		}
		else{
			isValided = false;
		}
	}
}

bool Omega::getIsValided(){
	return isValided;
}

float Omega::getOmega(int index){
	return _Omega[index];
}

void Omega::setOmega(int index, float value){
	_Omega[index] = value;
}

float Omega::getRawOmega(int index){
	return _RawOmega[index];
}





