/*
 * MathTools.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <MathTools.h>
#include <math.h>
#include <inttypes.h>
#include <stdio.h>
#include <Leds.h>
#include <AdditionalTools.h>

float MathTools::Sqrt(float x)
{
	float xhalf = 0.5f * x;

  union
  {
	float x;
    int i;
  } u;

  u.x = x;
  u.i = 0x5f3759df - (u.i >> 1);
  return x * u.x*(1.5f - xhalf * u.x * u.x);
}

float MathTools::Sign(float value){

	return value / fabs(value);

}

float MathTools::DegreeToRadian(float degree){
	return degree * RADIAN_PER_DEGREE;
}

float MathTools::RadianToDegree(float radian){
	return radian * DEGREE_PER_RADIAN;
}

float MathTools::Trim(float lowerBound, float value, float upperBound){

	return (value > upperBound ? upperBound : value < lowerBound ? lowerBound : value);

}

float MathTools::CutOff(float value, float BoundValue, float Bound){

	return ((value < Bound && value > -Bound) ? BoundValue : value);

}

float MathTools::QuadRoot(float value){
	return sqrt(sqrt(value));
}

float MathTools::OctRoot(float value){
	return sqrt(sqrt(sqrt(value)));
}

float MathTools::TrimResolution(float value){
	union {
		uint32_t d;
		float f;
	} x;
	x.f = value;
	x.d &= 0xfffffff0;
	return x.f;
}
