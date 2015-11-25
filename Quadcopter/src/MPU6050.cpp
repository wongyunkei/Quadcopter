/*
 * mpu6050.cpp
 *
 *  Created on: 2014¦~8¤ë23¤é
 *      Author: YunKei
 */

#include <MPU6050.h>
#include <I2C.h>
#include <Task.h>
#include <stdio.h>
#include <math.h>
#include <MathTools.h>
#include <Acceleration.h>

MPU6050* _mMPU6050[6];

void MPU6050::setI2CBypass(int index, bool onState){
	uint8_t data = 0;
	if(onState){
		data = 0x02;
	}
	Ticks::getInstance()->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_INT_PIN_CFG,data)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}
}

MPU6050::MPU6050(int index, I2C* i2c) : isValided(false){

	_mMPU6050[index] = this;
	i2cx = i2c;
	RawAccScale[0] = 2.0*GRAVITY / (RAWACCPOSX - RAWACCNEGX);
	RawAccScale[1] = 2.0*GRAVITY / (RAWACCPOSY - RAWACCNEGY);
	RawAccScale[2] = 2.0*GRAVITY / (RAWACCPOSZ - RAWACCNEGZ);
	RawAccOffset[0] = RAWACCPOSX * RawAccScale[0] - GRAVITY;
	RawAccOffset[1] = RAWACCPOSY * RawAccScale[1] - GRAVITY;
	RawAccOffset[2] = RAWACCPOSZ * RawAccScale[2] - GRAVITY;
	FastInitialization();

	RawOmegaOffset[0] = 0.3f;
	RawOmegaOffset[1] = -2.5f;
	RawOmegaOffset[2] = -0.9f;
}

void MPU6050::FastInitialization(){

	Ticks::getInstance()->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_PWR_MGMT_1,0x00)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}

	Ticks::getInstance()->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_SMPLRT_DIV,0x07)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}
	Ticks::getInstance()->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_CONFIG,0x00)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}
	Ticks::getInstance()->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_GYRO_CONFIG,0x18)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}
	Ticks::getInstance()->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_ACCEL_CONFIG,0x18)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}
}

MPU6050* MPU6050::getInstance(int index){

	return _mMPU6050[index];
}

bool MPU6050::Update(){

	uint8_t data[14];
	int16_t temp;

	if(!i2cx->BurstRead(ADDRESS, RA_ACCEL_XOUT_H, 14, data)){
		FastInitialization();
		isValided = false;
		return false;
	}

	for(int i = 0; i < 14; i += 2){
		if(i >= 0 && i <= 5){
			int j = i / 2;
			temp = (data[i + 1] | (data[i] << 8));
			RawAcc[j] = (float)temp * 0.0047884f;
		}
		else if(i >= 6 && i <= 7){
			temp = (data[i + 1] | (data[i] << 8));
			temperature = (float)temp / 340.0f + 36.53f;
		}
		else if(i >= 8 && i <= 13){
			temp = data[i + 1] | (data[i] << 8);
			RawOmega[(i - 8) / 2] = (float)temp * 0.0609756f;//0.0076335877862595f;
		}
	}

	float swap;

	swap = RawAcc[0];
	RawAcc[0] = -RawAcc[1];
	RawAcc[1] = -swap;
	RawAcc[2] = -RawAcc[2];

	swap = RawOmega[0];
	RawOmega[0] = RawOmega[1];
	RawOmega[1] = swap;

	for(int i = 0; i < 3; i++){
		RawAcc[i] *= RawAccScale[i];
		RawAcc[i] -= RawAccOffset[i];
//		RawOmega[i] -= getGyroTemperatureCompensation(i, temperature);
		RawOmega[i] -= RawOmegaOffset[i];
//		RawOmega[i] = MathTools::CutOff(RawOmega[i], 0.0f, 1.0f);
	}
	isValided = true;
	return true;
}

float MPU6050::getGyroTemperatureCompensation(int index, float temp){
	float value = 0;
	switch(index){
		case 0:
			value = -0.00007f*temp*temp + 0.0375*temp - 1.6842f;
			break;
		case 1:
			value = 0.0005f*temp*temp - 0.1131f*temp + 6.102f;
			break;
		case 2:
			value = 0.00002f*temp*temp + 0.0072f*temp - 1.2311f;
			break;
	}
	return value;
}

bool MPU6050::getIsValided(){
	return isValided;
}

void MPU6050::setTemperature(float value){
	temperature = value;
}

float MPU6050::getTemperature(){
	return temperature;
}

void MPU6050::setRawOmegaOffset(int channel, float value){
	RawOmegaOffset[channel] = value;
}

float MPU6050::getRawOmegaOffset(int channel){
	return RawOmegaOffset[channel];
}

void MPU6050::setRawOmega(int channel, float value){
	RawOmega[channel] = value;
}

float MPU6050::getRawOmega(int channel){
	return RawOmega[channel];
}

void MPU6050::setRawAcc(int channel, float value){
	RawAcc[channel] = value;
}

float MPU6050::getRawAcc(int channel){
	return RawAcc[channel];
}
