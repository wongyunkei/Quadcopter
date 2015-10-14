/*
 * Leds.cpp
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#include <Leds.h>
#include <Task.h>
#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

Leds* _mLeds;

Leds::Leds(){
	Leds_GPIO = GPIOD;
	uint32_t Leds_RCC = RCC_AHB1Periph_GPIOD;
	GPIO_InitTypeDef  GPIO_InitStructure;
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(Leds_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Leds_GPIO, &GPIO_InitStructure);
	_mLeds = this;
}

Leds* Leds::getInstance(){
	return _mLeds;
}

void Leds::LedsControl(LEDS leds , bool onState){

	switch(leds){

		case LED1:
			GPIO_WriteBit(Leds_GPIO, GPIO_Pin_0, onState ? Bit_SET : Bit_RESET);
			break;

		case LED2:
			GPIO_WriteBit(Leds_GPIO, GPIO_Pin_1, onState ? Bit_SET : Bit_RESET);
			break;

		case LED3:
			GPIO_WriteBit(Leds_GPIO, GPIO_Pin_2, onState ? Bit_SET : Bit_RESET);
			break;

		case LED4:
			GPIO_WriteBit(Leds_GPIO, GPIO_Pin_3, onState ? Bit_SET : Bit_RESET);
			break;

		default:
			break;
	}
}
GPIO_TypeDef* Leds::getLedsGPIO(){
	return Leds_GPIO;
}

void Blink_Led1(){
	GPIO_ToggleBits(Leds::getInstance()->getLedsGPIO(), GPIO_Pin_0);
}

void Blink_Led2(){
	GPIO_ToggleBits(Leds::getInstance()->getLedsGPIO(), GPIO_Pin_1);
}

void Blink_Led3(){
	GPIO_ToggleBits(Leds::getInstance()->getLedsGPIO(), GPIO_Pin_2);
}

void Blink_Led4(){
	GPIO_ToggleBits(Leds::getInstance()->getLedsGPIO(), GPIO_Pin_3);
}

void Leds::Toggle(LEDS leds){
	if(leds == LED1){
		GPIO_ToggleBits(Leds_GPIO, GPIO_Pin_0);
	}
	else if(leds == LED2){
		GPIO_ToggleBits(GPIOD, GPIO_Pin_1);
	}
	else if(leds == LED3){
		GPIO_ToggleBits(GPIOD, GPIO_Pin_2);
	}
	else if(leds == LED4){
		GPIO_ToggleBits(GPIOD, GPIO_Pin_3);
	}
}

void Leds::Blink(uint16_t period, LEDS leds, bool onState, int count){

	switch(leds){

		case LED1:
			if(onState){
				if(count >= 0){
					Task::getInstance()->Attach(period, 0, Blink_Led1, false, 2 * count);
				}
				else{
					Task::getInstance()->Attach(period, 0, Blink_Led1, true);
				}
			}
			else{
				Task::getInstance()->DeAttach(Blink_Led1);
			}
			break;

		case LED2:
			if(onState){
				if(count >= 0){
					Task::getInstance()->Attach(period, 0, Blink_Led2, false, 2 * count);
				}
				else{
					Task::getInstance()->Attach(period, 0, Blink_Led2, true);
				}
			}
			else{

				Task::getInstance()->DeAttach(Blink_Led2);
			}
			break;

		case LED3:
			if(onState){
				if(count >= 0){
					Task::getInstance()->Attach(period, 0, Blink_Led3, false, 2 * count);
				}
				else{
					Task::getInstance()->Attach(period, 0, Blink_Led3, true);
				}
			}
			else{
				Task::getInstance()->DeAttach(Blink_Led3);
			}
			break;

		case LED4:
			if(onState){
				if(count >= 0){
					Task::getInstance()->Attach(period, 0, Blink_Led4, false, 3 * count);
				}
				else{
					Task::getInstance()->Attach(period, 0, Blink_Led4, true);
				}
			}
			else{
				Task::getInstance()->DeAttach(Blink_Led4);
			}
			break;
		default:
			break;
	}
}
