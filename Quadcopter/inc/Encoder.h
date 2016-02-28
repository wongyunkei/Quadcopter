/*
 * Encoder.h
 *
 *  Created on: 2016¦~2¤ë23¤é
 *      Author: wongy
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stm32f4xx.h>
#include <stm32f4xx_tim.h>
#include <Configuration.h>
#include <AdditionalTools.h>

using namespace System;

namespace Sensors{
	class Encoder{
		public:
			class EncoderConfiguration{
				public:
					enum TimerSelections{
						TimerConf1,
						TimerConf2,
						TimerConf3,
						TimerConf4,
						TimerConf5,
						TimerConf6};
					EncoderConfiguration(Configuration* signalA, Configuration* signalB, TimerSelections timerConf);
					Configuration* _signalA;
					Configuration* _signalB;
					TimerSelections _timerConf;
			};
			Encoder(EncoderConfiguration* conf, float scale, float interval);
			void Poll();
			float ReadVel();
			float ReadPos();
		private:
			EncoderConfiguration* Conf;
			TIM_TypeDef* TIMx;
			float Interval;
			float Vel;
			float Pos;
			float Scale;
	};
}

#endif /* ENCODER_H_ */
