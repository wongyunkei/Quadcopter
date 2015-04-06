/*
 * Leds.cpp
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#include <Leds.h>
#include <Task.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

Leds* _mLeds;

Leds::Leds(){

	GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	_mLeds = this;
}

Leds* Leds::getInstance(){
	return _mLeds;
}

void Leds::LedsControl(LEDS leds , bool onState){

	switch(leds){

		case LED1:
			GPIO_WriteBit(GPIOA, GPIO_Pin_4, onState ? Bit_SET : Bit_RESET);
			break;

		case LED2:
			GPIO_WriteBit(GPIOA, GPIO_Pin_5, onState ? Bit_SET : Bit_RESET);
			break;

		default:
			break;
	}
}

void Blink_Led1(){
	GPIO_ToggleBits(GPIOA, GPIO_Pin_4);
}

void Blink_Led2(){
	GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
}

void Leds::Toggle(LEDS leds){
	if(leds == LED1){
		GPIO_ToggleBits(GPIOA, GPIO_Pin_4);
	}
	else if(leds == LED2){
		GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
	}
}

void Leds::Blink(uint16_t period, LEDS leds, bool onState){

	switch(leds){

		case LED1:
			if(onState){

				Task::getInstance()->Attach(period, 0, Blink_Led1, true, -1);
			}
			else{

				Task::getInstance()->DeAttach(Blink_Led1);
			}
			break;

		case LED2:
			if(onState){

				Task::getInstance()->Attach(period, 0, Blink_Led2, true, -1);
			}
			else{

				Task::getInstance()->DeAttach(Blink_Led2);
			}
			break;

		default:
			break;
	}
}
