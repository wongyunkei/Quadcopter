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
#include <Buzzer.h>

#define MPU6050_I2C			I2C2

MPU6050* _mMPU6050;

void UpdateTask(){
	MPU6050::getInstance()->Update();
}

MPU6050::MPU6050(float interval) : Interval(interval), inited(0), isValided(false){

	_mMPU6050 = this;

	RawAccScale[0] = 2.0*GRAVITY / (RAWACCPOSX - RAWACCNEGX);
	RawAccScale[1] = 2.0*GRAVITY / (RAWACCPOSY - RAWACCNEGY);
	RawAccScale[2] = 2.0*GRAVITY / (RAWACCPOSZ - RAWACCNEGZ);
	RawAccOffset[0] = RAWACCPOSX * RawAccScale[0] - GRAVITY;
	RawAccOffset[1] = RAWACCPOSY * RawAccScale[1] - GRAVITY;
	RawAccOffset[2] = RAWACCPOSZ * RawAccScale[2] - GRAVITY;
	FastInitialization();

//	Task::getInstance()->Attach(interval * 1000, 0, UpdateTask, false, 1024);
//	Task::getInstance()->Run();
//
//	while(!GyroCal());

	RawOmegaOffset[0] = 3.45f;//2.75;
	RawOmegaOffset[1] = 6.4;//5.7;//4.8;
	RawOmegaOffset[2] = -0.5;//-0.4;//1.33;
	inited = 1;
	Task::getInstance()->Attach(2, 0, UpdateTask, false, 32);
	Task::getInstance()->Run();
}

void MPU6050::FastInitialization(){

	Ticks::getInstance()->setTimeout(3);
	while(!I2C::getInstance(MPU6050_I2C)->Write(ADDRESS,RA_PWR_MGMT_1,0x00)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}
	Ticks::getInstance()->setTimeout(3);
	while(!I2C::getInstance(MPU6050_I2C)->Write(ADDRESS,RA_SMPLRT_DIV,0x07)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}
	Ticks::getInstance()->setTimeout(3);
	while(!I2C::getInstance(MPU6050_I2C)->Write(ADDRESS,RA_CONFIG,0x00)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}
	Ticks::getInstance()->setTimeout(3);
	while(!I2C::getInstance(MPU6050_I2C)->Write(ADDRESS,RA_GYRO_CONFIG,0x18)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}
	Ticks::getInstance()->setTimeout(3);
	while(!I2C::getInstance(MPU6050_I2C)->Write(ADDRESS,RA_ACCEL_CONFIG,0x18)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}
}

MPU6050* MPU6050::getInstance(){

	return _mMPU6050;
}

void GyroCalTask(){

	MPU6050::getInstance()->Update();
	for(int i = 0; i < 3; i++){
		MPU6050::getInstance()->setRawOmegaOffset(i, MPU6050::getInstance()->getRawOmegaOffset(i) + MPU6050::getInstance()->getRawOmega(i));
	}
}

bool MPU6050::GyroCal(){

	RawOmegaOffset[0] = 0;
	RawOmegaOffset[1] = 0;
	RawOmegaOffset[2] = 0;
	Task::getInstance()->Attach(Interval * 1000, 0, GyroCalTask, false, 10000);
	Task::getInstance()->Run();
	for(int i = 0; i < 3; i++){
		setRawOmegaOffset(i, getRawOmegaOffset(i) / 10000.0);
	}
	float offset[3] = {(float)getRawOmegaOffset(0), (float)getRawOmegaOffset(1), (float)getRawOmegaOffset(2)};
	printf("OmegaOffset:%g,%g,%g\r\n", offset[0], offset[1], offset[2]);
	return true;
}

bool MPU6050::Update(){

	uint8_t data[14];
	int16_t temp;

	if(!I2C::getInstance(MPU6050_I2C)->BurstRead(ADDRESS, RA_ACCEL_XOUT_H, 14, data)){
		FastInitialization();
		isValided = false;
		return false;
	}

	for(int i = 0; i < 14; i += 2){
		if(i >= 0 && i <= 5){
			int j = i / 2;
			temp = (data[i + 1] | (data[i] << 8));
			RawAcc[j] = (float)temp * 0.0047884;
		}
		else if(i >= 8 && i <= 13){
			temp = data[i + 1] | (data[i] << 8);
			RawOmega[(i - 8) / 2] = (float)temp * 0.0609756;
		}
	}

	float swap;

	swap = RawAcc[0];
	RawAcc[0] = RawAcc[1];
	RawAcc[1] = -swap;

	swap = RawOmega[0];
	RawOmega[0] = RawOmega[1];
	RawOmega[1] = -swap;

	for(int i = 0; i < 3; i++){
		RawAcc[i] *= RawAccScale[i];
		RawAcc[i] -= RawAccOffset[i];
		RawOmega[i] -= inited * RawOmegaOffset[i];
		RawOmega[i] = MathTools::CutOff(RawOmega[i], 0.0f, 1.0);
	}
	isValided = true;
	return true;
}

bool MPU6050::getIsValided(){
	return isValided;
}

void MPU6050::setRawOmegaOffset(int index, float value){
	RawOmegaOffset[index] = value;
}

float MPU6050::getRawOmegaOffset(int index){
	return RawOmegaOffset[index];
}

void MPU6050::setRawOmega(int index, float value){
	RawOmega[index] = value;
}

float MPU6050::getRawOmega(int index){
	return RawOmega[index];
}

void MPU6050::setRawAcc(int index, float value){
	RawAcc[index] = value;
}

float MPU6050::getRawAcc(int index){
	return RawAcc[index];
}
