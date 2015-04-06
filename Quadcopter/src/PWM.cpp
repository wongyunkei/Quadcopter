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

PWM* _mPWM;

PWM* PWM::getInstant(){
	return _mPWM;
}

PWM::PWM() : MaxPWM(10000), LowerLimit(7350), UpperLimit(25200), PWM_TIM(TIM8){

	_mPWM = this;

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

	TIM_TimeBaseStructure.TIM_Prescaler = 16 - 1;
	TIM_TimeBaseStructure.TIM_Period = 52500-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = LowerLimit;

	TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
	TIM_OC3Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
	TIM_OC4Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(PWM_TIM, ENABLE);
	TIM_Cmd(PWM_TIM, ENABLE);
	TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);

}

void PWM::Control(int index, double dutyCycle){
	uint16_t value = LowerLimit;
	if(dutyCycle > 10000 || dutyCycle < 0.0){
		return;
	}

	else{
		value += (uint16_t)((UpperLimit - LowerLimit) * dutyCycle / 10000);
		value = value > UpperLimit ? UpperLimit : value;
	}
	switch(index){
		case 0:
			TIM_SetCompare1(PWM_TIM, value);
			break;
		case 1:
			TIM_SetCompare2(PWM_TIM, value);
			break;
		case 2:
			TIM_SetCompare3(PWM_TIM, value);
			break;
		case 3:
			TIM_SetCompare4(PWM_TIM, value);
			break;
	}
}


double PWM::getLowerLimit(){
	return LowerLimit;
}

double PWM::getUpperLimit(){
	return UpperLimit;
}
