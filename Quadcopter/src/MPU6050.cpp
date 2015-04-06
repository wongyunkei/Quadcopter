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

#define MPU6050_I2C			I2C2

MPU6050* _mMPU6050;

void UpdateTask(){
	MPU6050::getInstance()->Update();
}

MPU6050::MPU6050(double interval) : Interval(interval), inited(0){

	_mMPU6050 = this;

	RawAccOffset[0] = 0.26;
	RawAccOffset[1] = 0.26;
	RawAccOffset[2] = 0.528387534;
	RawAccScale[0] = 0.986367685;
	RawAccScale[1] = 0.982351206;
	RawAccScale[2] = 0.960433939;

	FastInitialization();

//	Task::getInstance()->Attach(interval * 1000, 0, UpdateTask, false, 512);
//	Task::getInstance()->Run();

//	while(!GyroCal());

	RawOmegaOffset[0] = 0;
	RawOmegaOffset[1] = 4.8;
	RawOmegaOffset[2] = 1.33;
	inited = 1;
	Task::getInstance()->Attach(2, 0, UpdateTask, false, 32);
	Task::getInstance()->Run();
}

void MPU6050::FastInitialization(){

	while(!I2C::getInstance(MPU6050_I2C)->Write(ADDRESS,RA_PWR_MGMT_1,0x03));
	while(!I2C::getInstance(MPU6050_I2C)->Write(ADDRESS,RA_SMPLRT_DIV,0x01));
	while(!I2C::getInstance(MPU6050_I2C)->Write(ADDRESS,RA_CONFIG,0x00));
	while(!I2C::getInstance(MPU6050_I2C)->Write(ADDRESS,RA_GYRO_CONFIG,0x18));
	while(!I2C::getInstance(MPU6050_I2C)->Write(ADDRESS,RA_ACCEL_CONFIG,0x18));
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
	Task::getInstance()->Attach(Interval * 1000, 0, GyroCalTask, false, 512);
	Task::getInstance()->Run();
	for(int i = 0; i < 3; i++){
		setRawOmegaOffset(i, getRawOmegaOffset(i) / 512);
	}
	printf("OmegaOffset:%f,%f,%f\r\n", getRawOmegaOffset(0), getRawOmegaOffset(1), getRawOmegaOffset(2));
	return true;
}

bool MPU6050::Update(){

	uint8_t data[14];
	int16_t temp;

	if(!I2C::getInstance(MPU6050_I2C)->BurstRead(ADDRESS, RA_ACCEL_XOUT_H, 14, data)){
		FastInitialization();
		return false;
	}

	for(int i = 0; i < 14; i += 2){
		if(i >= 0 && i <= 5){
			int j = i / 2;
			temp = (data[i + 1] | (data[i] << 8));
			RawAcc[j] = (double)temp * 0.0047884;
		}
		else if(i >= 8 && i <= 13){
			temp = data[i + 1] | (data[i] << 8);
			RawOmega[(i - 8) / 2] = (double)temp * 0.0609756;
		}
	}

	double swap;

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
		RawOmega[i] = MathTools::CutOff(RawOmega[i], 0.0f, 5.0);
	}

	return true;
}

void MPU6050::setRawOmegaOffset(int index, double value){
	RawOmegaOffset[index] = value;
}

double MPU6050::getRawOmegaOffset(int index){
	return RawOmegaOffset[index];
}

void MPU6050::setRawOmega(int index, double value){
	RawOmega[index] = value;
}

double MPU6050::getRawOmega(int index){
	return RawOmega[index];
}

void MPU6050::setRawAcc(int index, double value){
	RawAcc[index] = value;
}

double MPU6050::getRawAcc(int index){
	return RawAcc[index];
}
