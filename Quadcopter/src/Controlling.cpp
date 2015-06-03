/*
 * Controlling.cpp
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#include <Controlling.h>
#include <Communicating.h>
#include <Quaternion.h>
#include <math.h>
#include <MathTools.h>
#include <stdio.h>
#include <Pid.h>
#include <PWM.h>
#include <Omega.h>
#include <Vector.h>
#include <Task.h>
#include <Battery.h>
#include <Acceleration.h>
#include <Sonic.h>
#include <PhasesMonitoring.h>
#include <Buzzer.h>
#include <Fuzzy.h>
#include <SE3.h>
#include <Eigen/Eigen>

using Eigen::Vector3f;

Controlling* _mControlling;

Controlling* Controlling::getInstant(){
	return _mControlling;
}

int startCount;

void StartingTask(){
	if(Controlling::getInstant()->getStarting()){
		if(startCount == 0){
			for(int i = 0; i < 4; i++){
				GPIO_WriteBit(GPIOC, GPIO_Pin_0, Bit_RESET);
				PWM::getInstant()->Control(i, INIT_PWM);
				Controlling::getInstant()->setLift(0);
				Controlling::getInstant()->setTarget(3, 0.6f);
//				PWM::getInstant()->Control(i, Controlling::getInstant()->RPM2PWM(i, INIT_RPM));
			}
		}
		if(startCount++ >= 25){

			startCount = 0;
			Controlling::getInstant()->setLift(0);
//			float rpm0 = PhasesMonitoring::getInstance()->getRPM(0);
//			float rpm1 = PhasesMonitoring::getInstance()->getRPM(1);
//			float rpm2 = PhasesMonitoring::getInstance()->getRPM(2);
//			float rpm3 = PhasesMonitoring::getInstance()->getRPM(3);
			//if(rpm0 > 0 && rpm1 > 0 && rpm2 > 0 && rpm3 > 0){
				Controlling::getInstant()->setStart(true);
				Communicating::getInstant()->RFSend(4,1);
			//}
//			else{
//				Controlling::getInstant()->setStart(false);
//				Buzzer::getInstance()->Frequency(10, 1000, true);
//				Controlling::getInstant()->Stopping();
//				Communicating::getInstant()->RFSend(4,0);
//			}
			Controlling::getInstant()->setStarting(false);
		}
	}
}

int StoppingDelayCount;

void StoppingTask(){
	if(Controlling::getInstant()->getStopping()){
//		if(StoppingDelayCount++ <= 50){
//			Controlling::getInstant()->setInitRPM(Controlling::getInstant()->getInitRPM() - 5);
//		}
//		else if(StoppingDelayCount++ <= 100){
//			Controlling::getInstant()->setInitRPM(Controlling::getInstant()->getInitRPM() - 3);
//		}
//		else if(StoppingDelayCount++ <= 200){
//			Controlling::getInstant()->setInitRPM(Controlling::getInstant()->getInitRPM() - 1);
//		}
		if(SE3::getInstance()->getPos()(2) > 0.25f){
			Controlling::getInstant()->setTarget(3, SE3::getInstance()->getPos()(2) - 0.001f);
			Controlling::getInstant()->setMaxLift(LANDING_MAX_LIFT);
		}
		else{
			StoppingDelayCount = 0;
			for(int i = 0; i < 4; i++){
				PWM::getInstant()->Control(i, 0);
			}
			Controlling::getInstant()->setStart(false);
			Pid::getInstance(0)->clear();
			Pid::getInstance(1)->clear();
			Pid::getInstance(2)->clear();
			Pid::getInstance(3)->clear();
			Pid::getInstance(4)->clear();
			Pid::getInstance(5)->clear();
			Pid::getInstance(6)->clear();

			Pid::getInstance(8)->clear();
			Pid::getInstance(9)->clear();
			Pid::getInstance(10)->clear();
			Pid::getInstance(11)->clear();
			Controlling::getInstant()->setLift(0);
			Controlling::getInstant()->setMaxLift(MAX_LIFT);
			for(int i = 0; i < 4; i++){
				Controlling::getInstant()->setMotorTarget(i, 0);
				Controlling::getInstant()->setMotorValue(i, 0);
			}
			Controlling::getInstant()->setStopping(false);
			Communicating::getInstant()->RFSend(4,0);
			GPIO_WriteBit(GPIOC, GPIO_Pin_0, Bit_SET);
		}
	}
}

Controlling::Controlling() : minLift(MIN_LIFT), maxLift(MAX_LIFT), initRPM(INIT_RPM), preFzPWM(0), XYPidDelayCount(0), HightPidDelayCount(0), started(false), starting(false), stopping(false), watchDogCount(0), initPWM(INIT_PWM), FzPWM(0), cosRollcosPitch(1), Lift(0.0f){
	_mControlling = this;
	ControlPWM = new PWM();
	target[0] = 0.0f;
	target[1] = 0.0f;
	target[2] = 0.0f;
	target[3] = 0.6f;
	FzPWM = target[3];
	thrust[0] = 0;
	thrust[1] = 0;
	thrust[2] = 0;
	thrust[3] = 0;
	offset[0] = 0.0;
	offset[1] = 0.0;
	offset[2] = 0.0;
	offset[3] = 0.0;
	RPYOffset[0] = 0.0f;
	RPYOffset[1] = 0.0f;
	RPYOffset[2] = 15.0f;

	rotorPWM[0] = initPWM;
	rotorPWM[1] = initPWM;
	rotorPWM[2] = initPWM;
	rotorPWM[3] = initPWM;

	MotorTarget[0] = INIT_RPM;
	MotorTarget[1] = INIT_RPM;
	MotorTarget[2] = INIT_RPM;
	MotorTarget[3] = INIT_RPM;

	MotorRPM[0] = 0;
	MotorRPM[1] = 0;
	MotorRPM[2] = 0;
	MotorRPM[3] = 0;

	MotorValue[0] = 0;
	MotorValue[1] = 0;
	MotorValue[2] = 0;
	MotorValue[3] = 0;

	RPYPid[0] = new Pid(0,1000.0f,2000.0f,0.0,6000.0f,0.002f);//(0,0.027,0.0001,0.0,0.002);
	RPYPid[1] = new Pid(1,2000.0f,2000.0f,0.0,6000.0f,0.002f);//(1,0.027,0.0001,0,0.002);//(1,0.0135,0.0005,0.00001,0.002);
	RPYPid[2] = new Pid(2,2500.0f,0.0f,0.0,6000.0f,0.002f);//(2,0.12,0.001,0.0,0.002);
	D_RPYPid[0] = new Pid(17,300.0f,0,0,6000.0f,0.002f);
	D_RPYPid[1] = new Pid(18,300.0f,0,0,6000.0f,0.002f);
	D_RPYPid[2] = new Pid(19,300.0f,0,0,6000.0f,0.002f);//(5,0.15,0,0.0,0.002);

	HightPid = new Pid(3, 2.0f, 0.0f, 7.0f, 6000.0f, 0.1f);

	XYPid[0] = new Pid(12, 0.0f, 0.0f, 0.0f, 6000.0f, 0.008f);
	XYPid[1] = new Pid(13, 0.0f, 0.0f, 0.0f, 6000.0f, 0.008f);

	float _fuzzyTable[15][15] = {{7, 6, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 6, 7},
								{6, 6, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 6, 6},
								{5, 4, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 4, 5},
								{4, 4, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 4, 4},
								{3, 2, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 2, 3},
								{2, 2, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 2, 2},
								{1, 1, 1, 1, -1, -4, -6, -7, -6, -4, -1, 1, 1, 1, 1},
								{0, 0, 0, 0, -1, -4, -6, -7, -6, -4, -1, 0, 0, 0, 0},
								{1, 1, 1, 1, -1, -4, -6, -7, -6, -4, -1, 1, 1, 1, 1},
								{2, 2, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 2, 2},
								{3, 2, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 2, 3},
								{4, 4, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 4, 4},
								{5, 4, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 4, 5},
								{6, 6, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 6, 6},
								{7, 6, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 6, 7}};

	fuzzyTable = new float*[15];
	for(int i = 0; i < 15; i++){
		fuzzyTable[i] = new float[15];
		for(int j = 0; j < 15; j++){
			fuzzyTable[i][j] = _fuzzyTable[i][j];
		}
	}

	FuzzyRPY[0] = new Fuzzy(0, 7, 0.7, 20.0, 1.0, -0.7, -20.0, 0.9, 800.0, 0.05, 2000.0, fuzzyTable);
	FuzzyRPY[1] = new Fuzzy(1, 7, 0.7, 20.0, 1.0, -0.7, -20.0, 0.9, 900.0, 0.05, 2000.0, fuzzyTable);
	FuzzyRPY[2] = new Fuzzy(2, 7, 0.7, 20.0, 1.0, -0.7, -20.0, 0.9, 2000.0, 0.1, 1000.0, fuzzyTable);

	MotorPid[0] = new Pid(8,0.8,0,0.0,0,0.002);
	MotorPid[1] = new Pid(9,0.8,0,0.0,0,0.002);
	MotorPid[2] = new Pid(10,0.8,0,0.0,0.0,0.002);
	MotorPid[3] = new Pid(11,0.8,0,0.0,0.0,0.002);

	Task::getInstance()->Attach(40, 0, StartingTask, true, -1);
	Task::getInstance()->Attach(40, 0, StoppingTask, true, -1);
}

void Controlling::setStarting(bool value){
	starting = value;
}

bool Controlling::getStarting(){
	return starting;
}
void Controlling::setStopping(bool value){
	stopping = value;
}

void Controlling::setMotorTarget(int index, float value){
	MotorTarget[index] = value;
}

float Controlling::getMotorTarget(int index){
	return MotorTarget[index];
}

void Controlling::MotorControl(int index, float value){
	float rpm = value + MotorPid[index]->pid(value, PhasesMonitoring::getInstance()->getRPM(index));
	PWM::getInstant()->Control(index, RPM2PWM(index, rpm));
}

float Controlling::RPM2PWM(int index, float rpm){
	if(rpm < INIT_RPM){
		return INIT_PWM;
	}
	float value;
	switch(index){
		case 0:
			value = 0.000164914797995569f*rpm*rpm+0.197209757423632f*rpm+150.004794708134f;
			break;
		case 1:
			value = 0.000141171946287363f*rpm*rpm+0.360672579944736f*rpm-129.496099527621f;
			break;
		case 2:
			value = 0.000162765820815333f*rpm*rpm+0.230788291687393f*rpm+41.3681489073706f;
			break;
		case 3:
			value = 0.000153235784108517f*rpm*rpm+0.366260783305642f*rpm-190.765711869109f;
			break;
	}
	return value < 0.0 ? INIT_PWM : value;
}

void Controlling::ControllingPoll(){

	if(started){
		if(watchDogCount < WATCHDOGCOUNT_LIMIT){
			watchDogCount++;
		}

		float k[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
		if(FuzzyRPY[0]->FuzzyAlgorithm(Quaternion::getInstance()->getEuler(0) + Quaternion::getInstance()->getInitAngles(0), MathTools::DegreeToRadian(Omega::getInstance()->getOmega(0)), &k[0][0], &k[0][1], &k[0][2])){

			RPYPid[0]->setPid(k[0][0], k[0][1], 0.0);
			D_RPYPid[0]->setPid(k[0][2], 0.0, 0.0);
		}
		if(FuzzyRPY[1]->FuzzyAlgorithm(Quaternion::getInstance()->getEuler(1) + Quaternion::getInstance()->getInitAngles(1), MathTools::DegreeToRadian(Omega::getInstance()->getOmega(1)), &k[1][0], &k[1][1], &k[1][2])){

			RPYPid[1]->setPid(k[1][0], k[1][1], 0.0);
			D_RPYPid[1]->setPid(k[1][2], 0.0, 0.0);
		}
		if(FuzzyRPY[2]->FuzzyAlgorithm(Quaternion::getInstance()->getEuler(2) + Quaternion::getInstance()->getInitAngles(2), MathTools::DegreeToRadian(Omega::getInstance()->getOmega(2)), &k[2][0], &k[2][1], &k[2][2])){

			RPYPid[2]->setPid(k[2][0], k[2][1], 0.0);
			D_RPYPid[2]->setPid(k[2][2], 0.0, 0.0);
		}
		errXY[0] = 0;
		errXY[1] = 0;
		if(XYPidDelayCount++ % 4 == 0){
			XYPidDelayCount = 0;
			errXY[0] = XYPid[0]->pid(0.0f, SE3::getInstance()->getPos()(0));
			errXY[1] = XYPid[1]->pid(0.0f, SE3::getInstance()->getPos()(1));
		}

		errRPY[0] = RPYPid[0]->pid(MathTools::DegreeToRadian(target[0] + RPYOffset[0]), Quaternion::getInstance()->getEuler(0)) - Quaternion::getInstance()->getInitAngles(0) + D_RPYPid[0]->pid(0, MathTools::DegreeToRadian(Omega::getInstance()->getOmega(0)));
		errRPY[1] = RPYPid[1]->pid(MathTools::DegreeToRadian(target[1] + RPYOffset[1]), Quaternion::getInstance()->getEuler(1)) - Quaternion::getInstance()->getInitAngles(1) + D_RPYPid[1]->pid(0, MathTools::DegreeToRadian(Omega::getInstance()->getOmega(1)));
		errRPY[2] = RPYPid[2]->pid(MathTools::DegreeToRadian(target[2] + RPYOffset[2]), Quaternion::getInstance()->getEuler(2)) + D_RPYPid[2]->pid(0, MathTools::DegreeToRadian(Omega::getInstance()->getOmega(2)));

		cosRollcosPitch = cosf(Quaternion::getInstance()->getEuler(0)) * cosf(Quaternion::getInstance()->getEuler(1));

		if(HightPidDelayCount++ % 50 == 0){
			HightPidDelayCount = 0;
			Vector3f pos = SE3::getInstance()->getPos();
			Lift += HightPid->pid(target[3], pos(2));
//			Lift *= fabsf(Lift);
//			Lift += minLift;
			Lift = MathTools::Trim(minLift, Lift, maxLift);
		}


//		MotorTarget[0] = initRPM / cosRollcosPitch + errRPY[0] - errRPY[1] + errRPY[2];
//		MotorTarget[1] = initRPM / cosRollcosPitch - errRPY[0] - errRPY[1] - errRPY[2];
//		MotorTarget[2] = initRPM / cosRollcosPitch - errRPY[0] + errRPY[1] + errRPY[2];
//		MotorTarget[3] = initRPM / cosRollcosPitch + errRPY[0] + errRPY[1] - errRPY[2];
		MotorTarget[0] = Lift / cosRollcosPitch + errRPY[0] - errRPY[1] + errRPY[2] - errXY[0] - errXY[1];
		MotorTarget[1] = Lift / cosRollcosPitch - errRPY[0] - errRPY[1] - errRPY[2] - errXY[0] + errXY[1];
		MotorTarget[2] = Lift / cosRollcosPitch - errRPY[0] + errRPY[1] + errRPY[2] + errXY[0] + errXY[1];
		MotorTarget[3] = Lift / cosRollcosPitch + errRPY[0] + errRPY[1] - errRPY[2] + errXY[0] - errXY[1];

		for(int i = 0; i < 4; i++){
			PWM::getInstant()->Control(i, MotorTarget[i]);
			//MotorControl(i, MotorTarget[i]);
		}
	}

	if(watchDogCount >= WATCHDOGCOUNT_LIMIT ||
			fabsf(target[0] + RPYOffset[0] - MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(0)/* - Quaternion::getInstance()->getInitAngles(0)*/)) > 20.0f ||
			fabsf(target[1] + RPYOffset[1] - MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(1)/* - Quaternion::getInstance()->getInitAngles(1)*/)) > 20){// || ((Sonic::getInstance()->getDistance() - 1.2) > 0)){
//	if(watchDogCount >= WATCHDOGCOUNT_LIMIT || fabsf(target[0] + RPYOffset[0] - MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(0))) > 15.0f || fabsf(target[1] + RPYOffset[1] - MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(1))) > 15){// || ((Sonic::getInstance()->getDistance() - 1.2) > 0)){

		if(started){
			Buzzer::getInstance()->Frequency(10, 100, true);
			Stopping();
		}
	}
}

void Controlling::Starting(){

	Communicating::getInstant()->setCmdData(Battery::getInstance()->getBatteryLevel());
	if(Battery::getInstance()->getBatteryLevel() > 12.0){
		if(!started){
			startCount = 0;
			stopping = false;
			starting = true;
		}
		else{
			for(int i = 0; i < 4; i++){
				PWM::getInstant()->Control(i, 0);
			}
			startCount = 0;
			started = false;
			stopping = false;
			starting = false;
		}
	}
	else{
		Usart::getInstance(USART1)->Print("Low Battery!\n");
	}
}

bool Controlling::getStopping(){
	return stopping;
}

void Controlling::setStart(bool value){
	started = value;
}
bool Controlling::getStart(){
	return started;
}

float Controlling::getMotorValue(int index){
	return MotorValue[index];
}

void Controlling::setMotorValue(int index, float value){
	MotorValue[index] = value;
}

float Controlling::getMotorRPM(int index){
	return MotorRPM[index];
}

void Controlling::setMotorRPM(int index, float value){
	MotorRPM[index] = value;
}

void Controlling::Stopping(){
	if(!stopping){
		stopping = true;
		starting = false;
	}
}

float Controlling::getThrust(int index){
	return thrust[index];
}

float Controlling::getRotorPWM(int index){
	return rotorPWM[index];
}

float Controlling::getTarget(int index){
	return target[index];
}

void Controlling::setTarget(int index, float value){
	target[index] = value;
}

float Controlling::getOffset(int index){
	return offset[index];
}

void Controlling::setOffset(int index, float value){
	offset[index] = value;
}

void Controlling::setRPYOffset(int index, float value){
	RPYOffset[index] = value;
}

float Controlling::getRPYOffset(int index){
	return RPYOffset[index];
}

void Controlling::clearWatchDogCount(){
	watchDogCount = 0;
}

float Controlling::getErrRPY(int index){
	return errRPY[index];
}

float Controlling::getInitPWM(){
	return initPWM;
}

void Controlling::setInitPWM(float value){
	initPWM = value;
}


float Controlling::getInitRPM(){
	return initRPM;
}

void Controlling::setInitRPM(float value){
	initRPM = value;
}

float Controlling::getLift(){
	return Lift;
}

void Controlling::setLift(float value){
	Lift = value;
}

float Controlling::getFzPWM(){
	return FzPWM;
}

void Controlling::setFzPWM(float value){
	FzPWM = value;
}

float Controlling::getPreFzPWM(){
	return preFzPWM;
}

void Controlling::setPreFzPWM(float value){
	preFzPWM = value;
}

void Controlling::setRotorPWM(int index, float value){
	rotorPWM[index] = value;
}

void Controlling::setMaxLift(float value){
	maxLift = value;
}

void Controlling::setMinLift(float value){
	minLift = value;
}
