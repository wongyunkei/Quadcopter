/*
 * Omega.cpp
 *
 *  Created on: 2014�~8��24��
 *      Author: YunKei
 */

#include <Omega.h>

Omega::Omega(MPU6050* mMPU6050) : _mMPU6050(mMPU6050), isValided(false){
}

void Omega::Update(){

	if(_mMPU6050->getIsValided()){
		_Omega = _mMPU6050->getRawOmega();
		isValided = true;
	}
	else{
		isValided = false;
	}
}

bool Omega::getIsValided(){
	return isValided;
}

Vector3f Omega::getOmega(){
	return _Omega;
}

void Omega::setOmega(Vector3f value){
	_Omega = value;
}
