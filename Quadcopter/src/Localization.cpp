/*
 * Localization.cpp
 *
 *  Created on: 2016¦~2¤ë26¤é
 *      Author: wongy
 */

#include <Localization.h>

Localization::Localization(Quaternion* quaternion,
		Encoder* encoderX,
		Encoder* encoderY,
		float  encoderYTranslation,
		float  encoderXTranslation,
		float interval) : mQuaternion(quaternion),
		mEncoderX(encoderX),
		mEncoderY(encoderY),
		EncoderXTranslation(encoderXTranslation),
		EncoderYTranslation(encoderYTranslation),
		Interval(interval), EncoderXFramePosX(encoderXTranslation), EncoderXFramePosY(encoderYTranslation){
	Pos.setZero();
	Vel.setZero();
}

Vector3f Localization::ReadPos(){
	return Pos;
}

Vector3f Localization::ReadVel(){
	return Vel;
}

void Localization::LocalizationCalc(){
	float yaw = mQuaternion->getEuler()[2];
	float sinyaw = sinf(yaw);
	float cosyaw = cosf(yaw);
	float x = mEncoderX->ReadVel();
	float y = mEncoderY->ReadVel();
	EncoderXFramePosX += (cosyaw * x - sinyaw * y) * Interval;
	EncoderXFramePosY += (sinyaw * x + cosyaw * y) * Interval;
	Vector3f t;
	t << EncoderXFramePosX - cosyaw * EncoderXTranslation + sinyaw * EncoderYTranslation  , EncoderXFramePosY - sinyaw * EncoderXTranslation - cosyaw * EncoderYTranslation, 0;
	Vel = (t - Pos) / Interval;
	Pos = t;
}

void Localization::setEncoderXTranslation(float value){
	EncoderXTranslation = value;
}

void Localization::setEncoderYTranslation(float value){
	EncoderYTranslation = value;
}

void Localization::Reset(){
	Pos.setZero();
	Vel.setZero();
	EncoderXFramePosX = EncoderXTranslation;
	EncoderXFramePosY = EncoderYTranslation;
}
