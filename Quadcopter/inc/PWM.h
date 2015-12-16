/*
 * PWM.h
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#ifndef PWM_H_
#define PWM_H_

#include <stm32f4xx.h>
#include <stm32f4xx_tim.h>
#include <Configuration.h>

using namespace System;

namespace Control{
	class PWM{
		public:
			class PWMConfiguration{
				public:
					PWMConfiguration(Configuration* pwm1, uint8_t pwmSource1, Configuration* pwm2, uint8_t pwmSource2, Configuration* pwm3, uint8_t pwmSource3, Configuration* pwm4, uint8_t pwmSource4, float freq);
					Configuration* _pwm1;
					Configuration* _pwm2;
					Configuration* _pwm3;
					Configuration* _pwm4;
					uint8_t _pwmSource1;
					uint8_t _pwmSource2;
					uint8_t _pwmSource3;
					uint8_t _pwmSource4;
					float _freq;
				private:
			};
			PWM(PWMConfiguration* conf);
			void Control1(double dutyCycle);
			void Control2(double dutyCycle);
			void Control3(double dutyCycle);
			void Control4(double dutyCycle);
			double getLowerLimit();
			double getUpperLimit();
		private:
			PWMConfiguration* Conf;
			double LowerLimit;
			double UpperLimit;
			double MaxPWM;
	};
};

#endif /* PWM_H_ */
