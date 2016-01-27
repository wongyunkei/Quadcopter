/*
 * Controlling.cpp
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#include <Controlling.h>

using Eigen::Vector3f;

using namespace Math;
using namespace Utility;

void StartingTask(){

	if(App::mApp->mControlling->getStarting()){

		if(App::mApp->mControlling->startCount == 0){
			App::mApp->mControlling->_mPWM->Control1(App::mApp->mControlling->initLift);
			App::mApp->mControlling->_mPWM->Control2(App::mApp->mControlling->initLift);
			App::mApp->mControlling->_mPWM->Control3(App::mApp->mControlling->initLift);
			App::mApp->mControlling->_mPWM->Control4(App::mApp->mControlling->initLift);
		}
		else if(App::mApp->mControlling->startCount >= 15){
			App::mApp->mControlling->setStart(true);
			App::mApp->mCommunicating1->Acknowledgement();
		}
		else if(App::mApp->mControlling->startCount >= 40){
			App::mApp->mControlling->setStarting(false);
		}
		App::mApp->mControlling->startCount++;
	}
}

void StoppingTask(){
	if(App::mApp->mControlling->getStopping()){
		if(App::mApp->mControlling->Lift > App::mApp->mControlling->leadingLift){
			App::mApp->mControlling->Lift = App::mApp->mControlling->Lift - 500;
		}
		else if(App::mApp->mControlling->StoppingDelayCount < 25){
			App::mApp->mControlling->StoppingDelayCount++;
		}
		else{
			App::mApp->mControlling->StoppingDelayCount = 0;
			App::mApp->mControlling->StopAllMotors();
			App::mApp->mControlling->setStart(false);
			App::mApp->mControlling->RollPid->clear();
			App::mApp->mControlling->PitchPid->clear();
			App::mApp->mControlling->YawPid->clear();
			App::mApp->mControlling->KdRollPid->clear();
			App::mApp->mControlling->KdPitchPid->clear();
			App::mApp->mControlling->KdYawPid->clear();
			App::mApp->mControlling->Lift = 0;
			App::mApp->mControlling->setStopping(false);
		}
		App::mApp->mCommunicating1->Acknowledgement();
	}
}

Controlling::Controlling(PWM* mPWM) : _mPWM(mPWM), WatchDogLimit(800), leadingLift(5000), minLift(7000), maxLift(9500),
		started(false), starting(false), stopping(false),
		watchDogCount(0), Lift(3000.0f), RollTarget(0),
		PitchTarget(0), YawTarget(0), RollOffset(0),
		PitchOffset(0), YawOffset(0), initLift(5000), startCount(0), StoppingDelayCount(0){

	RollPid = new Pid(60000.0f,500000.0f,0.0,500.0f,0.002f);
	PitchPid = new Pid(60000.0f,500000.0f,0.0,500.0f,0.002f);
	YawPid = new Pid(60000.0f,500000.0f,0.0,500.0f,0.002f);

	KdRollPid = new Pid(4000.0f,0.0f,0.0,10000.0f,0.002f);
	KdPitchPid = new Pid(4000.0f,0.0f,0.0,10000.0f,0.002f);
	KdYawPid = new Pid(4000.0f,0.0f,0.0,10000.0f,0.002f);

	App::mApp->mTask->Attach(40, 0, StartingTask, true);
	App::mApp->mTask->Attach(40, 0, StoppingTask, true);
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

void Controlling::ControllingPoll(){

	if(started){
		if(watchDogCount < WatchDogLimit){
			watchDogCount++;
		}
		float errRoll = RollPid->pid(MathTools::DegreeToRadian(RollTarget), App::mApp->mQuaternion->getEuler()[0] - MathTools::DegreeToRadian(RollOffset)) + KdRollPid->pid(0, App::mApp->mOmega->getOmega()[0]);
		float errPitch = PitchPid->pid(MathTools::DegreeToRadian(PitchTarget), App::mApp->mQuaternion->getEuler()[1] - MathTools::DegreeToRadian(PitchOffset)) +  + KdPitchPid->pid(0, App::mApp->mOmega->getOmega()[1]);
		float errYaw = YawPid->pid(MathTools::DegreeToRadian(YawTarget), App::mApp->mQuaternion->getEuler()[2] - MathTools::DegreeToRadian(YawOffset)) +  + KdYawPid->pid(0, App::mApp->mOmega->getOmega()[2]);

		float cosRollcosPitch = cosf(App::mApp->mQuaternion->getEuler()[0] - MathTools::DegreeToRadian(RollOffset)) * cosf(App::mApp->mQuaternion->getEuler()[1] - MathTools::DegreeToRadian(PitchOffset));

		float Motor1PWM = Lift / cosRollcosPitch + errRoll - errPitch + errYaw;
		float Motor2PWM = Lift / cosRollcosPitch - errRoll - errPitch - errYaw;
		float Motor3PWM = Lift / cosRollcosPitch - errRoll + errPitch + errYaw;
		float Motor4PWM = Lift / cosRollcosPitch + errRoll + errPitch - errYaw;
		if(stopping){
			Motor1PWM = MathTools::Trim(leadingLift, Motor1PWM, maxLift);
			Motor2PWM = MathTools::Trim(leadingLift, Motor2PWM, maxLift);
			Motor3PWM = MathTools::Trim(leadingLift, Motor3PWM, maxLift);
			Motor4PWM = MathTools::Trim(leadingLift, Motor4PWM, maxLift);
		}
		else{
			Motor1PWM = MathTools::Trim(minLift, Motor1PWM, maxLift);
			Motor2PWM = MathTools::Trim(minLift, Motor2PWM, maxLift);
			Motor3PWM = MathTools::Trim(minLift, Motor3PWM, maxLift);
			Motor4PWM = MathTools::Trim(minLift, Motor4PWM, maxLift);
		}

		_mPWM->Control1(Motor1PWM);
		_mPWM->Control2(Motor2PWM);
		_mPWM->Control3(Motor3PWM);
		_mPWM->Control4(Motor4PWM);

		float integral1 = RollPid->getIntegral();
		float integral2 = PitchPid->getIntegral();
		float integral3 = YawPid->getIntegral();

		if(App::mApp->mCommunicating1->PrintType == 1){
			AdditionalTools::setBuffer(0, &integral1, 1);
			AdditionalTools::setBuffer(1, &integral2, 1);
			AdditionalTools::setBuffer(2, &integral3, 1);
		}
	}

	if(watchDogCount >= WatchDogLimit ||
	   fabsf(RollTarget + RollOffset - MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[0])) > 25.0f ||
	   fabsf(PitchTarget + PitchOffset - MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[1])) > 25.0f){
		if(started){
			Stopping();
			App::mApp->mCommunicating1->Acknowledgement();
		}
	}
}

void Controlling::Starting(){

	if(4096 * 0.52 / App::mApp->mADC->getReading() > 3.8f){
		if(!started){
			startCount = 0;
			stopping = false;
			starting = true;
		}
		else{
			StopAllMotors();
			startCount = 0;
			started = false;
			stopping = false;
			starting = false;
		}
	}
	else{
		App::mApp->mCommunicating1->Acknowledgement();
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

void Controlling::Stopping(){
	if(!stopping){
		stopping = true;
		starting = false;
	}
}

float Controlling::getRollTarget(){
	return RollTarget;
}

float Controlling::getPitchTarget(){
	return PitchTarget;
}

float Controlling::getYawTarget(){
	return YawTarget;
}

void Controlling::setRollTarget(float roll){
	RollTarget = roll;
}

void Controlling::setPitchTarget(float pitch){
	PitchTarget = pitch;
}

void Controlling::setYawTarget(float yaw){
	YawTarget = yaw;
}

float Controlling::getRollOffset(){
	return RollOffset;
}

float Controlling::getPitchOffset(){
	return PitchOffset;
}

float Controlling::getYawOffset(){
	return YawOffset;
}

void Controlling::setRollOffset(float roll){
	RollOffset = roll;
}

void Controlling::setPitchOffset(float pitch){
	PitchOffset = pitch;
}

void Controlling::setYawOffset(float yaw){
	YawOffset = yaw;
}

void Controlling::clearWatchDogCount(){
	watchDogCount = 0;
}

void Controlling::StopAllMotors(){
	_mPWM->Control1(0);
	_mPWM->Control2(0);
	_mPWM->Control3(0);
	_mPWM->Control4(0);
}
