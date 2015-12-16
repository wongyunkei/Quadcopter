/*
 * PWM.cpp
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#include <PWM.h>
#include <inttypes.h>
#include <stdio.h>
#include <Controlling.h>

using namespace Control;

PWM::PWMConfiguration::PWMConfiguration(Configuration* pwm1, uint8_t pwmSource1, Configuration* pwm2, uint8_t pwmSource2, Configuration* pwm3, uint8_t pwmSource3, Configuration* pwm4, uint8_t pwmSource4, float freq) : _pwm1(pwm1), _pwmSource1(pwmSource1), _pwm2(pwm2), _pwmSource2(pwmSource2), _pwm3(pwm3), _pwmSource3(pwmSource3), _pwm4(pwm4), _pwmSource4(pwmSource4), _freq(freq){
};

PWM::PWM(PWMConfiguration* conf) : Conf(conf), MaxPWM(10000), LowerLimit(0), UpperLimit(168000000.0 / conf->_freq - 1){

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	RCC_AHB1PeriphClockCmd(conf->_pwm1->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_pwm2->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_pwm3->_rcc, ENABLE);
	RCC_AHB1PeriphClockCmd(conf->_pwm4->_rcc, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = conf->_pwm1->_pin;
	GPIO_Init(conf->_pwm1->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_pwm2->_pin;
	GPIO_Init(conf->_pwm2->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_pwm3->_pin;
	GPIO_Init(conf->_pwm3->_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = conf->_pwm4->_pin;
	GPIO_Init(conf->_pwm4->_port, &GPIO_InitStructure);

	GPIO_PinAFConfig(conf->_pwm1->_port, conf->_pwmSource1, GPIO_AF_TIM8);
	GPIO_PinAFConfig(conf->_pwm2->_port, conf->_pwmSource2, GPIO_AF_TIM8);
	GPIO_PinAFConfig(conf->_pwm3->_port, conf->_pwmSource3, GPIO_AF_TIM8);
	GPIO_PinAFConfig(conf->_pwm4->_port, conf->_pwmSource4, GPIO_AF_TIM8);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_Period = (uint32_t)(168000000.0 / conf->_freq - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = LowerLimit;

	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_Cmd(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	Control1(0);
	Control2(0);
	Control3(0);
	Control4(0);
}

void PWM::Control1(double dutyCycle){
	uint16_t value = LowerLimit;
	if(dutyCycle > 10000 || dutyCycle < 0){
		return;
	}

	else{
		value += (uint16_t)((UpperLimit - LowerLimit) * dutyCycle / 10000.0);
		value = value > UpperLimit ? UpperLimit : value;
	}
	TIM_SetCompare1(TIM8, value);
}

void PWM::Control2(double dutyCycle){
	uint16_t value = LowerLimit;
	if(dutyCycle > 10000 || dutyCycle < 0){
		return;
	}

	else{
		value += (uint16_t)((UpperLimit - LowerLimit) * dutyCycle / 10000);
		value = value > UpperLimit ? UpperLimit : value;
	}
	TIM_SetCompare2(TIM8, value);
}

void PWM::Control3(double dutyCycle){
	uint16_t value = LowerLimit;
	if(dutyCycle > 10000 || dutyCycle < 0){
		return;
	}

	else{
		value += (uint16_t)((UpperLimit - LowerLimit) * dutyCycle / 10000);
		value = value > UpperLimit ? UpperLimit : value;
	}
	TIM_SetCompare3(TIM8, value);
}

void PWM::Control4(double dutyCycle){
	uint16_t value = LowerLimit;
	if(dutyCycle > 10000 || dutyCycle < 0){
		return;
	}

	else{
		value += (uint16_t)((UpperLimit - LowerLimit) * dutyCycle / 10000);
		value = value > UpperLimit ? UpperLimit : value;
	}
	TIM_SetCompare4(TIM8, value);
}

double PWM::getLowerLimit(){
	return LowerLimit;
}

double PWM::getUpperLimit(){
	return UpperLimit;
}
