/*
 * Sonic.cpp
 *
 *  Created on: 2014¦~12¤ë18¤é
 *      Author: YunKei
 */

#include <math.h>
#include <Sonic.h>
#include <MathTools.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_it.h>
#include <stm32f4xx_tim.h>
#include <Delay.h>
#include <Usart.h>
#include <stdio.h>
#include <Kalman.h>

Sonic* _mSonic;

Sonic::Sonic() : distance(0){
	_mSonic = this;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
	TIM_TimeBaseStructure.TIM_Period = 10000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0xf;

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE ;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);
	double R[2] = {0.0001, -1};
	SonicKalman = new Kalman(0.000001, R, 0, 1);
}

Kalman* Sonic::getSonicKalman(){
	return SonicKalman;
}

Sonic* Sonic::getInstance(){
	return _mSonic;
}

void Sonic::setDistance(double value){
	distance = value;
}

double Sonic::getDistance(){
	return distance;
}

void Sonic::Update(){
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
	Delay::DelayUS(10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);
}

void TIM2_IRQHandler(void){

	if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == SET){
			TIM_SetCounter(TIM2, 0);
		}
		else{
			double value = TIM_GetCapture2(TIM2);
			double temp = 0;
			Sonic::getInstance()->getSonicKalman()->Filtering(&temp, value / 5800.0, 0.0);
			Sonic::getInstance()->setDistance(temp);
		}
	}
}

