/*
 * Pid.cpp
 *
 *  Created on: 2014¦~11¤ë8¤é
 *      Author: YunKei
 */

#include <App.h>
#include <Pid.h>
#include <stdio.h>
#include <Ticks.h>
#include <MathTools.h>

Pid* _mPid[20];

Pid::Pid(int index, float kp, float ki, float kd, float integralLimit, float t) : Kp(kp), Ki(ki), Kd(kd), Integral(0), IntegralLimit(integralLimit), PreErr(0), DefaultPeriod(t), Period(t), PreTimeStamp(0){
	_mPid[index] = this;
}

Pid* Pid::getInstance(int index){
	return _mPid[index];
}

float Pid::setPid(float kp, float ki, float kd){
	Kp = kp;
	Ki = ki;
	Kd = kd;
}

float Pid::getPid(int index){
	float v = 0;

	switch(index){
		case 0:
			v = Kp;
			break;
		case 1:
			v = Ki;
			break;
		case 2:
			v = Kd;
			break;

	}

	return v;
}

void Pid::clear(){
	Integral = 0;
	PreErr = 0;
	PreTimeStamp = 0;
}

float Pid::pid(float target, float current){
	float t = App::mApp->mTicks->getTicks();
	t /= 1000.0f;
	if(PreTimeStamp > 0){
		Period = t < PreTimeStamp ? (10000 - PreTimeStamp + t) : t - PreTimeStamp;
	}
	else{
		Period = DefaultPeriod;
	}
	PreTimeStamp = t;
	float err = target - current;
	Integral += err;
	Integral *= Period;
	Integral = MathTools::Trim(-IntegralLimit, Integral, IntegralLimit);
	float derivative = err - PreErr;
	PreErr = err;
	return Kp * err + Ki * Integral + Kd * derivative / Period;
}

float Pid::getIntegral(){
	return Integral;
}
