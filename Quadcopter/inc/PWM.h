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

namespace Control{
	class PWM{
		public:
			PWM();
			static PWM* getInstant();
			void Control(int, double);
			double getLowerLimit();
			double getUpperLimit();
		private:
			double LowerLimit;
			double UpperLimit;
			double MaxPWM;
			TIM_TypeDef* PWM_TIM;
	};
};

using namespace Control;

#endif /* PWM_H_ */
