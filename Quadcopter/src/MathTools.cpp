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

double MathTools::Sign(double value){

	return value / fabs(value);

}

double MathTools::DegreeToRadian(double degree){
	return degree * RADIAN_PER_DEGREE;
}

double MathTools::RadianToDegree(double radian){
	return radian * DEGREE_PER_RADIAN;
}

double MathTools::Trim(double value, double lowerBound, double upperBound){

	return (value > upperBound ? upperBound : value < lowerBound ? lowerBound : value);

}

double MathTools::CutOff(double value, double BoundValue, double Bound){

	return ((value < Bound && value > -Bound) ? BoundValue : value);

}

double MathTools::QuadRoot(double value){
	return sqrt(sqrt(value));
}

double MathTools::OctRoot(double value){
	return sqrt(sqrt(sqrt(value)));
}
