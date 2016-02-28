/*
 * Encoder.cpp
 *
 *  Created on: 2016¦~2¤ë23¤é
 *      Author: wongy
 */

#include <Encoder.h>

using namespace Sensors;

Encoder::EncoderConfiguration::EncoderConfiguration(Configuration* signalA,
		Configuration* signalB,
		TimerSelections timerConf) : _signalA(signalA),
		_signalB(signalB),
		_timerConf(timerConf){
}

Encoder::Encoder(EncoderConfiguration* conf, float scale, float interval) : Conf(conf), Scale(scale), Interval(interval), Vel(0), Pos(0){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	uint8_t GPIO_AF_TIMx;
	uint32_t RCC_TIMx;
	uint8_t signalASource;
	uint8_t signalBSource;
	uint16_t TIM_Prescaler = 83;
	switch(conf->_timerConf){
		case EncoderConfiguration::TimerConf1:
			TIMx = TIM1;
			GPIO_AF_TIMx = GPIO_AF_TIM1;
			RCC_TIMx = RCC_APB2Periph_TIM1;
			TIM_Prescaler = 167;
			break;
		case EncoderConfiguration::TimerConf2:
			TIMx = TIM8;
			GPIO_AF_TIMx = GPIO_AF_TIM8;
			RCC_TIMx = RCC_APB2Periph_TIM8;
			TIM_Prescaler = 167;
			break;
		case EncoderConfiguration::TimerConf3:
			TIMx = TIM2;
			GPIO_AF_TIMx = GPIO_AF_TIM2;
			RCC_TIMx = RCC_APB1Periph_TIM2;
			break;
		case EncoderConfiguration::TimerConf4:
			TIMx = TIM3;
			GPIO_AF_TIMx = GPIO_AF_TIM3;
			RCC_TIMx = RCC_APB1Periph_TIM3;
			break;
		case EncoderConfiguration::TimerConf5:
			TIMx = TIM4;
			GPIO_AF_TIMx = GPIO_AF_TIM4;
			RCC_TIMx = RCC_APB1Periph_TIM4;
			break;
		case EncoderConfiguration::TimerConf6:
			TIMx = TIM5;
			GPIO_AF_TIMx = GPIO_AF_TIM5;
			RCC_TIMx = RCC_APB1Periph_TIM5;
			break;
	}

	for(int i = 0; i < 16; i++){
		if(conf->_signalA->_pin == _BV(i)){
			signalASource = i;
		}
		if(conf->_signalB->_pin == _BV(i)){
			signalBSource = i;
		}
	}

	if(conf->_timerConf < 2){
		RCC_APB2PeriphClockCmd(RCC_TIMx, ENABLE);
	}
	else{
		RCC_APB1PeriphClockCmd(RCC_TIMx, ENABLE);
	}

	RCC_AHB1PeriphClockCmd(conf->_signalA->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_signalB->_rcc, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = conf->_signalA->_pin;
	GPIO_Init(conf->_signalA->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_signalB->_pin;
	GPIO_Init(conf->_signalB->_port, &GPIO_InitStructure);

	GPIO_PinAFConfig(conf->_signalA->_port, signalASource, GPIO_AF_TIMx);
	GPIO_PinAFConfig(conf->_signalB->_port, signalBSource, GPIO_AF_TIMx);

	TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	TIM_SetCounter(TIMx, 32768);
	TIM_Cmd(TIMx, ENABLE);
}

void Encoder::Poll(){
	Vel = (int32_t)TIM_GetCounter(TIMx) - 32768;
	TIM_SetCounter(TIMx, 32768);
	Vel *= Scale;
	Pos += Vel;
	Vel /= Interval;
}

float Encoder::ReadVel(){
	return Vel;
}

float Encoder::ReadPos(){
	return Pos;
}
