/*
 * Controlling.cpp
 *
 *  Created on: 2014�~11��11��
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

Controlling* _mControlling;

Controlling* Controlling::getInstant(){
	return _mControlling;
}

int startCount;

void StartingTask(){
	if(Controlling::getInstant()->getStarting()){
		if(startCount == 0){
			for(int i = 0; i < 4; i++){
				PWM::getInstant()->Control(i, INIT_PWM);
//				PWM::getInstant()->Control(i, Controlling::getInstant()->RPM2PWM(i, INIT_RPM));
			}
		}
		if(startCount++ >= 25){

			startCount = 0;
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
		if(StoppingDelayCount++ <= 50){
			Controlling::getInstant()->setInitRPM(Controlling::getInstant()->getInitRPM() - 5);
		}
		else if(StoppingDelayCount++ <= 100){
			Controlling::getInstant()->setInitRPM(Controlling::getInstant()->getInitRPM() - 3);
		}
		else if(StoppingDelayCount++ <= 200){
			Controlling::getInstant()->setInitRPM(Controlling::getInstant()->getInitRPM() - 1);
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

			for(int i = 0; i < 4; i++){
				Controlling::getInstant()->setMotorTarget(i, 0);
				Controlling::getInstant()->setMotorValue(i, 0);
			}
			Controlling::getInstant()->setStopping(false);
			Communicating::getInstant()->RFSend(4,0);
		}
	}
}

Controlling::Controlling() : initRPM(INIT_RPM), preFzPWM(0), HightPidDelayCount(0), started(false), starting(false), stopping(false), watchDogCount(0), initPWM(INIT_PWM), FzPWM(0), cosRollcosPitch(1){
	_mControlling = this;
	ControlPWM = new PWM();
	target[0] = 0.0;
	target[1] = 0.0;
	target[2] = 0.0;
	target[3] = 0.0;
	FzPWM = target[3];
	thrust[0] = 0;
	thrust[1] = 0;
	thrust[2] = 0;
	thrust[3] = 0;
	offset[0] = 0.0;
	offset[1] = 0.0;
	offset[2] = 0.0;
	offset[3] = 0.0;
	RPYOffset[0] = -1.0;
	RPYOffset[1] = -2.0;
	RPYOffset[2] = 0.0;

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

//	RPYPid[0] = new Pid(0,0.01,0.0001,0.0,1000.0,0.002);//(0,0.027,0.0001,0.0,0.002);
//	RPYPid[1] = new Pid(1,0.01,0.0001,0.0,1000.0,0.002);//(1,0.027,0.0001,0,0.002);//(1,0.0135,0.0005,0.00001,0.002);
//	RPYPid[2] = new Pid(2,0.01,0.0,0,1000.0,0.002);//(2,0.12,0.001,0.0,0.002);
//	D_RPYPid[0] = new Pid(3,0.01,0,0.0,1000.0,0.002);
//	D_RPYPid[1] = new Pid(4,0.01,0,0.0,1000.0,0.002);
//	D_RPYPid[2] = new Pid(5,0.001,0,0,1000.0,0.002);//(5,0.15,0,0.0,0.002);
//	RPYPid[0] = new Pid(0,33.45,1.052,0.0,10000.0,0.002);//(0,0.027,0.0001,0.0,0.002);
//	RPYPid[1] = new Pid(1,33.45,1.052,0.0,10000.0,0.002);//(1,0.027,0.0001,0,0.002);//(1,0.0135,0.0005,0.00001,0.002);
//	RPYPid[2] = new Pid(2,19.729,0.62,0,10000.0,0.002);//(2,0.12,0.001,0.0,0.002);
//	D_RPYPid[0] = new Pid(3,96.5,0,0.0,10000.0,0.002);
//	D_RPYPid[1] = new Pid(4,96.5,0,0.0,10000.0,0.002);
//	D_RPYPid[2] = new Pid(5,56.95,0,0,10000.0,0.002);//(5,0.15,0,0.0,0.002);

//	RPYPid[0] = new Pid(0,280,1.052,0.0,1000.0,0.002);//(0,0.027,0.0001,0.0,0.002);
//	RPYPid[1] = new Pid(1,280,1.052,0.0,1000.0,0.002);//(1,0.027,0.0001,0,0.002);//(1,0.0135,0.0005,0.00001,0.002);
//	RPYPid[2] = new Pid(2,19.729,0.62,0,1000.0,0.002);//(2,0.12,0.001,0.0,0.002);
//	D_RPYPid[0] = new Pid(3,96.5,0,0.0,1000.0,0.002);
//	D_RPYPid[1] = new Pid(4,96.5,0,0.0,1000.0,0.002);
//	D_RPYPid[2] = new Pid(5,56.95,0,0,1000.0,0.002);//(5,0.15,0,0.0,0.002);

//	HightPid = new Pid(6, 8, 0.0567281, 0.0, 4, 0.06);

	RPYPid[0] = new Pid(0,700,0.005,0.0,3000.0,0.002);//(0,0.027,0.0001,0.0,0.002);
	RPYPid[1] = new Pid(1,800,0.005,0.0,3000.0,0.002);//(1,0.027,0.0001,0,0.002);//(1,0.0135,0.0005,0.00001,0.002);
	RPYPid[2] = new Pid(2,1000,0.005,0.0,3000.0,0.002);//(2,0.12,0.001,0.0,0.002);
	D_RPYPid[0] = new Pid(17,200,0,0,3000.0,0.002);
	D_RPYPid[1] = new Pid(18,200,0,0,3000.0,0.002);
	D_RPYPid[2] = new Pid(19,100,0,0,3000.0,0.002);//(5,0.15,0,0.0,0.002);

	double _fuzzyTable[15][15] = {{7, 6, 2, 1, -1, -4, -6, -7, -6, -4, -1, 1, 2, 6, 7},
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

	fuzzyTable = new double*[15];
	for(int i = 0; i < 15; i++){
		fuzzyTable[i] = new double[15];
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

void Controlling::setMotorTarget(int index, double value){
	MotorTarget[index] = value;
}

double Controlling::getMotorTarget(int index){
	return MotorTarget[index];
}

void Controlling::MotorControl(int index, float value){
	double rpm = value + MotorPid[index]->pid(value, PhasesMonitoring::getInstance()->getRPM(index));
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

//	Communicating::getInstant()->RFSend(4, k[0][2]);
//	Communicating::getInstant()->RFSend(1, Controlling::getInstant()->_k[1]);
//	Communicating::getInstant()->RFSend(2, Controlling::getInstant()->_k[2]);
//	Communicating::getInstant()->RFSend(4, Fuzzy::getInstance(0)->u);
//	Communicating::getInstant()->RFSend(4, Fuzzy::getInstance(0)->E);
//	printf("%g,%g,%g\n", _k[0], _k[1], _k[2]);
//	_k[0] = k[1][0];
//	_k[1] = k[1][1];
//	_k[2] = k[1][2];
//	printf("%g,%g,%g\n", _k[0], _k[1], _k[2]);
//	_k[0] = k[2][0];
//	_k[1] = k[2][1];
//	_k[2] = k[2][2];
//	printf("%g,%g,%g\n\n", _k[0], _k[1], _k[2]);

	if(watchDogCount < WATCHDOGCOUNT_LIMIT && (started)){
		watchDogCount++;

		double k[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
		if(FuzzyRPY[0]->FuzzyAlgorithm(Quaternion::getInstance()->getEuler(0) - Quaternion::getInstance()->getInitAngles(0), MathTools::DegreeToRadian(Omega::getInstance()->getOmega(0)), &k[0][0], &k[0][1], &k[0][2])){

			RPYPid[0]->setPid(k[0][0], k[0][1], 0.0);
			D_RPYPid[0]->setPid(k[0][2], 0.0, 0.0);
		}
		if(FuzzyRPY[1]->FuzzyAlgorithm(Quaternion::getInstance()->getEuler(1) - Quaternion::getInstance()->getInitAngles(1), MathTools::DegreeToRadian(Omega::getInstance()->getOmega(1)), &k[1][0], &k[1][1], &k[1][2])){

			RPYPid[1]->setPid(k[1][0], k[1][1], 0.0);
			D_RPYPid[1]->setPid(k[1][2], 0.0, 0.0);
		}
		if(FuzzyRPY[2]->FuzzyAlgorithm(Quaternion::getInstance()->getEuler(2) - Quaternion::getInstance()->getInitAngles(2), MathTools::DegreeToRadian(Omega::getInstance()->getOmega(2)), &k[2][0], &k[2][1], &k[2][2])){

			RPYPid[2]->setPid(k[2][0], k[2][1], 0.0);
			D_RPYPid[2]->setPid(k[2][2], 0.0, 0.0);
		}


		errRPY[0] = RPYPid[0]->pid(MathTools::DegreeToRadian(target[0] + RPYOffset[0]), Quaternion::getInstance()->getEuler(0)) - Quaternion::getInstance()->getInitAngles(0) + D_RPYPid[0]->pid(0, MathTools::DegreeToRadian(Omega::getInstance()->getOmega(0)));
		errRPY[1] = RPYPid[1]->pid(MathTools::DegreeToRadian(target[1] + RPYOffset[1]), Quaternion::getInstance()->getEuler(1)) - Quaternion::getInstance()->getInitAngles(1) + D_RPYPid[1]->pid(0, MathTools::DegreeToRadian(Omega::getInstance()->getOmega(1)));
		errRPY[2] = RPYPid[2]->pid(MathTools::DegreeToRadian(target[2] + RPYOffset[2]), Quaternion::getInstance()->getEuler(2)) + D_RPYPid[2]->pid(0, MathTools::DegreeToRadian(Omega::getInstance()->getOmega(2)));

		cosRollcosPitch = cosf(Quaternion::getInstance()->getEuler(0)) * cosf(Quaternion::getInstance()->getEuler(1));

		MotorTarget[0] = initRPM / cosRollcosPitch + errRPY[0] - errRPY[1] + errRPY[2];
		MotorTarget[1] = initRPM / cosRollcosPitch - errRPY[0] - errRPY[1] - errRPY[2];
		MotorTarget[2] = initRPM / cosRollcosPitch - errRPY[0] + errRPY[1] + errRPY[2];
		MotorTarget[3] = initRPM / cosRollcosPitch + errRPY[0] + errRPY[1] - errRPY[2];

		for(int i = 0; i < 4; i++){
			PWM::getInstant()->Control(i, MotorTarget[i]);
			//MotorControl(i, MotorTarget[i]);
		}
	}

	if(watchDogCount >= WATCHDOGCOUNT_LIMIT || fabsf(MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(0) - Quaternion::getInstance()->getInitAngles(0))) > 10 || fabsf(MathTools::RadianToDegree(Quaternion::getInstance()->getEuler(1) - Quaternion::getInstance()->getInitAngles(1))) > 10){// || ((Sonic::getInstance()->getDistance() - 1.2) > 0)){

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

double Controlling::getMotorValue(int index){
	return MotorValue[index];
}

void Controlling::setMotorValue(int index, double value){
	MotorValue[index] = value;
}

double Controlling::getMotorRPM(int index){
	return MotorRPM[index];
}

void Controlling::setMotorRPM(int index, double value){
	MotorRPM[index] = value;
}

void Controlling::Stopping(){
	if(!stopping){
		stopping = true;
		starting = false;
	}
}

double Controlling::getThrust(int index){
	return thrust[index];
}

double Controlling::getRotorPWM(int index){
	return rotorPWM[index];
}

double Controlling::getTarget(int index){
	return target[index];
}

void Controlling::setTarget(int index, double value){
	target[index] = value;
}

double Controlling::getOffset(int index){
	return offset[index];
}

void Controlling::setOffset(int index, double value){
	offset[index] = value;
}

void Controlling::setRPYOffset(int index, double value){
	RPYOffset[index] = value;
}

double Controlling::getRPYOffset(int index){
	return RPYOffset[index];
}

void Controlling::clearWatchDogCount(){
	watchDogCount = 0;
}

double Controlling::getErrRPY(int index){
	return errRPY[index];
}

double Controlling::getInitPWM(){
	return initPWM;
}

void Controlling::setInitPWM(double value){
	initPWM = value;
}


double Controlling::getInitRPM(){
	return initRPM;
}

void Controlling::setInitRPM(double value){
	initRPM = value;
}


double Controlling::getFzPWM(){
	return FzPWM;
}

void Controlling::setFzPWM(double value){
	FzPWM = value;
}

double Controlling::getPreFzPWM(){
	return preFzPWM;
}

void Controlling::setPreFzPWM(double value){
	preFzPWM = value;
}

void Controlling::setRotorPWM(int index, double value){
	rotorPWM[index] = value;
}
