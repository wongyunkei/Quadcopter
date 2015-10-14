/*
 * HMC5883L.cpp
 *
 *  Created on: 2014¦~8¤ë23¤é
 *      Author: YunKei
 */

#include <HMC5883L.h>
#include <I2C.h>
#include <Task.h>
#include <stdio.h>
#include <math.h>
#include <MathTools.h>
#include <MPU6050.h>

#define HMC5883L_I2C	I2C2

HMC5883L* _mHMC5883L;

//void UpdateTask(){
//	HMC5883L::getInstance()->Update();
//}

HMC5883L::HMC5883L(float interval) : Interval(interval), inited(0), RawHeadOffset(0), isValided(false){

	_mHMC5883L = this;
	RawMagneticFieldScale[0] = XSCALE;
	RawMagneticFieldScale[1] = YSCALE;
	RawMagneticFieldScale[2] = ZSCALE;
	RawMagneticFieldOffset[0] = XOFFSET;
	RawMagneticFieldOffset[1] = YOFFSET;
	RawMagneticFieldOffset[2] = ZOFFSET;
	FastInitialization();
	float R[2] = {0.1, -1};
	CompassKalman = new Kalman(0.0001f, R, RawHead, 1.0f);
}

void HMC5883L::FastInitialization(){

//	MPU6050::getInstance()->setI2CBypass(true);

//	Ticks::getInstance()->setTimeout(3);
//	while(!I2C::getInstance(HMC5883L_I2C)->Write(ADDRESS,CFG_REG_A,0x18)){
//		if(Ticks::getInstance()->Timeout()){
//			return;
//		}
//	}
//	Ticks::getInstance()->setTimeout(3);
//	while(!I2C::getInstance(HMC5883L_I2C)->Write(ADDRESS,CFG_REG_B,0x00)){
//		if(Ticks::getInstance()->Timeout()){
//			return;
//		}
//	}
	Ticks::getInstance()->setTimeout(3);
	while(!I2C::getInstance(HMC5883L_I2C)->Write(ADDRESS,MODE_REG,0x00)){
		if(Ticks::getInstance()->Timeout()){
			return;
		}
	}

//	MPU6050::getInstance()->setI2CBypass(false);
//	Update();
	RawHeadOffset = RawHead;
}

HMC5883L* HMC5883L::getInstance(){

	return _mHMC5883L;
}

//void CompassCal(){
//
//	HMC5883L::getInstance()->Update();
//	for(int i = 0; i < 3; i++){
//		HMC5883L::getInstance()->setRawOmegaOffset(i, HMC5883L::getInstance()->getRawOmegaOffset(i) + HMC5883L::getInstance()->getRawOmega(i));
//	}
//}

bool HMC5883L::CompassCal(){

//	RawOmegaOffset[0] = 0;
//	RawOmegaOffset[1] = 0;
//	RawOmegaOffset[2] = 0;
//	Task::getInstance()->Attach(Interval * 1000, 0, GyroCalTask, false, 10000);
//	Task::getInstance()->Run();
//	for(int i = 0; i < 3; i++){
//		setRawOmegaOffset(i, getRawOmegaOffset(i) / 10000.0);
//	}
//	float offset[3] = {(float)getRawOmegaOffset(0), (float)getRawOmegaOffset(1), (float)getRawOmegaOffset(2)};
//	printf("OmegaOffset:%g,%g,%g\r\n", offset[0], offset[1], offset[2]);
	return true;
}

bool HMC5883L::Update(){

	uint8_t data[6];
	int16_t temp;

//	MPU6050::getInstance()->setI2CBypass(true);
	if(!I2C::getInstance(HMC5883L_I2C)->BurstRead(ADDRESS, DATA_OUT_X_MSB, 6, data)){
		FastInitialization();
		isValided = false;
		return false;
	}

	for(int i = 0; i < 6; i += 2){
		int j = i / 2;
		temp = (data[i + 1] | (data[i] << 8));
		RawMagneticField[j] = (float)temp * 0.73f;
	}

	float swap = RawMagneticField[0];
	RawMagneticField[0] = RawMagneticField[2];
	RawMagneticField[2] = RawMagneticField[1];
	RawMagneticField[1] = swap;

	for(int i = 0; i < 3; i++){
		RawMagneticField[i] *= RawMagneticFieldScale[i];
		RawMagneticField[i] -= RawMagneticFieldOffset[i];
	}
	float a = RawMagneticField[0] - RawMagneticField[1] * RawMagneticField[1];
	float b = -RawMagneticField[2] - 1 + RawMagneticField[1] * RawMagneticField[1];
	RawHead = atan2f(b, a) - RawHeadOffset;
	//printf("%g\n", MathTools::CalcLength(RawMagneticField, 3));
	CompassKalman->Filtering(&RawHead, RawHead, 0.0f);
	isValided = true;
//	MPU6050::getInstance()->setI2CBypass(false);
	return true;
}

bool HMC5883L::getIsValided(){
	return isValided;
}

//void HMC5883L::setRawOmegaOffset(int index, float value){
//	RawOmegaOffset[index] = value;
//}
//
//float HMC5883L::getRawOmegaOffset(int index){
//	return RawOmegaOffset[index];
//}
//
//void HMC5883L::setRawOmega(int index, float value){
//	RawOmega[index] = value;
//}
//
//float HMC5883L::getRawOmega(int index){
//	return RawOmega[index];
//}
//

void HMC5883L::setRawHead(float value){
	RawHead = value;
}

float HMC5883L::getRawHead(){
	return RawHead;
}

void HMC5883L::setRawMagneticField(int index, float value){
	RawMagneticField[index] = value;
}

float HMC5883L::getRawMagneticField(int index){
	return RawMagneticField[index];
}
