/*
 * Omega.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Omega.h>
#include <MPU6050.h>
#include <MathTools.h>

Omega* _mOmega[6];

Omega::Omega(int index) : DevIndex(index), isValided(false){
	_mOmega[index] = this;

	float R[3][2] = {{0.001f, -1},
					{0.001f, -1},
					{0.001f, -1}};
	OmegaKalman[0] = new Kalman(0.0001f, R[0], MPU6050::getInstance(0)->getRawOmega(0), 1);
	OmegaKalman[1] = new Kalman(0.0001f, R[1], MPU6050::getInstance(0)->getRawOmega(1), 1);
	OmegaKalman[2] = new Kalman(0.0001f, R[2], MPU6050::getInstance(0)->getRawOmega(2), 1);
}

Omega* Omega::getInstance(int index){
	return _mOmega[index];
}

void Omega::Update(){

	for(int i = 0; i < 3; i++){
		float temp = MPU6050::getInstance(DevIndex)->getRawOmega(i);
		if(MPU6050::getInstance(DevIndex)->getIsValided()){
			if(temp == temp){
				_Omega[i] = temp;
//				OmegaKalman[i]->Filtering(&_Omega[i], MPU6050::getInstance(0)->getRawOmega(i), 0.0f);
//				_Omega[i] = MathTools::CutOff(_Omega[i], 0.0f, 5.0f);
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

float Omega::getOmega(int channel){
	return _Omega[channel];
}

void Omega::setOmega(int channel, float value){
	_Omega[channel] = value;
}

float Omega::getRawOmega(int channel){
	return _RawOmega[channel];
}
