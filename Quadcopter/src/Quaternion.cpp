/*
 * Quaternion.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <App.h>
#include <Quaternion.h>
#include <MathTools.h>
#include <math.h>
#include <Kalman.h>
#include <MPU6050.h>
#include <stdio.h>
#include <Pid.h>
#include <Task.h>
#include <HMC5883L.h>
#include <Controlling.h>
#include <Delay.h>
#include <AdditionalTools.h>
#include <Communicating.h>
#include <Led.h>
#include <UART.h>

using namespace Utility;

Quaternion::Quaternion(Acceleration* mAcceleration, Omega* mOmega, Compass* mCompass, float interval) : _mAcceleration(mAcceleration), _mOmega(mOmega), _mCompass(mCompass), Interval(interval), Valid(false){
	Matrix4f Q;
	Q.setIdentity();
	Q *= 1e-12f;
	Matrix3f R;
	R.setIdentity();
	R *= 1e-7f;
	_Quaternion = EulerToQuaternion(mAcceleration->getAngle());
	_QuaternionKalman = new Kalman(_Quaternion, Q, R);
	Update();
}

bool Quaternion::Update(){
	if(_mOmega->getIsValided()){
		Vector3f omega = _mOmega->getOmega() * MathTools::RADIAN_PER_DEGREE;
		Matrix4f T = calcStateTransMatrix(omega, Interval);
		Vector4f q = T * _Quaternion;
		Vector3f E = QuaternionToEuler(q);
		bool valid = true;
		for(int i = 0; i < 3; i++){
			if(fabs(E[i]) > 0.5f){
				valid = false;
			}
		}
		bool AccValid = _mAcceleration->getIsValided() && _mAcceleration->getAcc().norm() > Acceleration::Gravity * 0.9f && _mAcceleration->getAcc().norm() < Acceleration::Gravity * 1.1f;
		Vector3f angle;
		bool MagValid = _mCompass->getIsValided() && _mCompass->getMag().norm() > 0.9f && _mCompass->getMag().norm() < 1.1f;

		if(AccValid){
			angle = _mAcceleration->getAngle();
			if(MagValid){
				angle[2] = _mCompass->getAngle()[2];
			}
			else{
				angle[2] = E[2];
			}
		}
		if(valid && AccValid){
			Eigen::Matrix<float, 3, 4> C = calcQuatToEulerMeasMatrix(q);
			Vector4f quat = _QuaternionKalman->Filtering(T, _Quaternion, C, angle) ? _QuaternionKalman->getCorrectedData() : T * _Quaternion;
			quat.normalize();
			for(int i = 0; i < 4; i++){
				if(quat[i] != quat[i]){
					Valid = false;
					return false;
				}
			}
			_Quaternion = quat;
		}
		else{
			_Quaternion = T * _Quaternion;
			_Quaternion.normalize();
			if(!valid && AccValid){
				angle = _mAcceleration->getFilteredAngle();
				if(MagValid){
					angle[2] = _mCompass->getFilteredAngle()[2];
				}
				else{
					angle[2] = E[2];
				}
				_Quaternion = EulerToQuaternion(angle);
			}
		}
		_Euler = QuaternionToEuler(_Quaternion);
		return true;
	}
	else{
		Valid = false;
		return false;
	}
}

Matrix4f Quaternion::calcStateTransMatrix(Vector3f w, float t){
	Matrix4f A;
	A << 0, -w[0], -w[1], -w[2],
		w[0], 0, w[2], -w[1],
		w[1], -w[2], 0, w[0],
		w[2], w[1], -w[0], 0;
	A *= 0.5f;
	Matrix4f T;
	T.setIdentity();
	T += A * t;
	return T;
}

Eigen::Matrix<float, 3, 4> Quaternion::calcQuatToEulerMeasMatrix(Vector4f q){
	Eigen::Matrix<float, 3, 4> C;
	Matrix3f R = QuaternionToMatrix(q);
	Eigen::Matrix<float, 3, 4> n;
	n << 2*R(2,2)*q[1],
		 2*(R(2,2)*q[0]+2*R(1,2)*q[1]),
		 2*(R(2,2)*q[3]+2*R(1,2)*q[2]),
		 2*R(2,2)*q[2],
		 2*q[2],
		 -2*q[3],
		 2*q[0],
		 -2*q[1],
		 2*R(0,0)*q[3],
		 2*R(0,0)*q[2],
		 2*(R(0,0)*q[0]+2*R(0,1)*q[2]),
		 2*(R(0,0)*q[0]+2*R(0,1)*q[3]);
	Eigen::Matrix<float, 3, 1> d;
	d << R(2,2)*R(2,2)+R(1,2)*R(1,2),
		 sqrt(1-R(0,2)*R(0,2)),
		 R(0,0)*R(0,0)+R(0,1)*R(0,1);
	C << -n(0,0)/d(0,0),-n(0,1)/d(0,0),-n(0,2)/d(0,0),-n(0,3)/d(0,0),
		 n(1,0)/d(1,0),n(1,1)/d(1,0),n(1,2)/d(1,0),n(1,3)/d(1,0),
		 -n(2,0)/d(2,0),-n(2,1)/d(2,0),-n(2,2)/d(2,0),-n(2,3)/d(2,0);
	return C;
}

Vector3f Quaternion::QuaternionToEuler(Vector4f q){
	Vector3f euler;
	euler[0] = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
	float r32 = 2 * (q[0] * q[1] + q[2] * q[3]);
	float r33 = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
	euler[1] = atan2f(-2 * (q[1] * q[3] - q[0] * q[2]), sqrtf(r32 * r32 + r33 * r33));
	euler[2] = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
	return euler;
}

Vector4f Quaternion::EulerToQuaternion(Vector3f euler){
	return FixedAnglesToQuaternion(MatrixToFixedAngles(EulerToMatrix(euler)));
}

Vector4f Quaternion::FixedAnglesToQuaternion(Vector3f fixedAngles){
	float CosHalfRoll = cosf(fixedAngles[0] / 2.0f), SinHalfRoll = sinf(fixedAngles[0] / 2.0f);
	float CosHalfPitch = cosf(fixedAngles[1] / 2.0f), SinHalfPitch = sinf(fixedAngles[1] / 2.0f);
	float CosHalfYaw = cosf(fixedAngles[2] / 2.0f), SinHalfYaw = sinf(fixedAngles[2] / 2.0f);
	Vector4f q;
	q[0] = CosHalfRoll * CosHalfPitch * CosHalfYaw - SinHalfRoll * SinHalfPitch * SinHalfYaw;
	q[1] = CosHalfRoll * SinHalfPitch * SinHalfYaw + SinHalfRoll * CosHalfPitch * CosHalfYaw;
	q[2] = CosHalfRoll * SinHalfPitch * CosHalfYaw - SinHalfRoll * CosHalfPitch * SinHalfYaw;
	q[3] = CosHalfRoll * CosHalfPitch * SinHalfYaw + SinHalfRoll * SinHalfPitch * CosHalfYaw;
	return q;
}

Matrix3f Quaternion::EulerToMatrix(Vector3f euler){
	Matrix3f R;
	float CosRoll = cosf(euler[0]), SinRoll = sinf(euler[0]);
	float CosPitch = cosf(euler[1]), SinPitch = sinf(euler[1]);
	float CosYaw = cosf(euler[2]), SinYaw = sinf(euler[2]);
	R(0,0) = CosYaw * CosPitch;
	R(0,1) = -SinYaw * CosRoll + CosYaw * SinPitch * SinRoll;
	R(0,2) = SinYaw * SinRoll + CosYaw * SinPitch * CosRoll;
	R(1,0) = SinYaw * CosPitch;
	R(1,1) = CosYaw * CosRoll + SinYaw * SinPitch * SinRoll;
	R(1,2) = -CosYaw * SinRoll + SinYaw * SinPitch * CosRoll;
	R(2,0) = -SinPitch;
	R(2,1) = CosPitch * SinRoll;
	R(2,2) = CosPitch * CosRoll;
	return R;
}

Vector3f Quaternion::MatrixToFixedAngles(Matrix3f R){
	Vector3f fixedAngles;
	fixedAngles[0] = atan2f(-R(1,2), R(2,2));
	fixedAngles[1] = atan2f(R(0,2), sqrtf(R(1,2)*R(1,2) + R(2,2)*R(2,2)));
	fixedAngles[2] = atan2f(-R(0,1), R(0,0));
	return fixedAngles;
}

Matrix3f Quaternion::QuaternionToMatrix(Vector4f q){
	Matrix3f M;
	M << 1-2*(q[2]*q[2]+q[3]*q[3]),
		 2*(q[1]*q[2]-q[0]*q[3]),
		 2*(q[1]*q[3]+q[0]*q[2]),
		 2*(q[1]*q[2]+q[0]*q[3]),
		 1-2*(q[1]*q[1]+q[3]*q[3]),
		 2*(q[2]*q[3]-q[0]*q[1]),
		 2*(q[1]*q[3]-q[0]*q[2]),
		 2*(q[2]*q[3]+q[0]*q[1]),
		 1-2*(q[1]*q[1]+q[2]*q[2]);
	return M;
}

Vector3f Quaternion::getEuler(){
	return _Euler;
}

void Quaternion::Reset(){
	Vector3f angle = _mAcceleration->getFilteredAngle();
	angle[2] = _mCompass->getFilteredAngle()[2];
	_Quaternion = EulerToQuaternion(angle);
	_QuaternionKalman->Clear(_Quaternion);
}
