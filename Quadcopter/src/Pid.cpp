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

Pid::Pid(float kp, float ki, float kd, float integralLimit, float t) : Kp(kp), Ki(ki), Kd(kd), Integral(0), IntegralLimit(integralLimit), PreErr(0), DefaultPeriod(t), Period(t), PreTimeStamp(0){
}

void Pid::setKp(float kp){
	Kp = kp;
}

void Pid::setKi(float ki){
	Ki = ki;
}

void Pid::setKd(float kd){
	Kd = kd;
}

float Pid::getKp(){
	return Kp;
}

float Pid::getKi(){
	return Ki;
}

float Pid::getKd(){
	return Kd;
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
