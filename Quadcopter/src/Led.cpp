/*
  * Led.cpp
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#include <Led.h>
#include <App.h>
#include <Config.h>
#include <Task.h>
#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

Led* LedAddr = 0;

Led::LedConfiguration::LedConfiguration(Configuration* led, BitAction onState) : _led(led), _onState(onState), _offState(onState ? Bit_RESET : Bit_SET){
};

Led::Led(LedConfiguration* conf) : Conf(conf){
	GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(conf->_led->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->_led->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(conf->_led->_port, &GPIO_InitStructure);

	LedControl(false);
}

void Led::LedControl(bool onState){
	GPIO_WriteBit(Conf->_led->_port, Conf->_led->_pin, onState ? Conf->_onState : Conf->_offState);
}

void BlinkTask1(){
	App::mApp->mLed1->Toggle();
}

void BlinkTask2(){
	App::mApp->mLed2->Toggle();
}

void BlinkTask3(){
	App::mApp->mLed3->Toggle();
}

void BlinkTask4(){
	App::mApp->mLed4->Toggle();
}

void Led::Toggle(){
	GPIO_ToggleBits(Conf->_led->_port, Conf->_led->_pin);
}

void Led::Blink(bool onState, uint16_t period, int count){
	Led* LedAddr = this;
	if(LedAddr == App::mApp->mLed1){
		if(onState){
			if(count >= 0){
				App::mApp->mTask->Attach(period / 2, 0, BlinkTask1, false, 2 * count);
			}
			else{
				App::mApp->mTask->Attach(period / 2, 0, BlinkTask1, true);
			}
		}
		else{
			App::mApp->mTask->DeAttach(BlinkTask1);
		}
	}
	else if(LedAddr == App::mApp->mLed2){
		if(onState){
			if(count >= 0){
				App::mApp->mTask->Attach(period / 2, 0, BlinkTask2, false, 2 * count);
			}
			else{
				App::mApp->mTask->Attach(period / 2, 0, BlinkTask2, true);
			}
		}
		else{
			App::mApp->mTask->DeAttach(BlinkTask2);
		}
	}
	else if(LedAddr == App::mApp->mLed3){
			if(onState){
				if(count >= 0){
					App::mApp->mTask->Attach(period / 2, 0, BlinkTask3, false, 2 * count);
				}
				else{
					App::mApp->mTask->Attach(period / 2, 0, BlinkTask3, true);
				}
			}
			else{
				App::mApp->mTask->DeAttach(BlinkTask3);
			}
	}
	else if(LedAddr == App::mApp->mLed4){
		if(onState){
			if(count >= 0){
				App::mApp->mTask->Attach(period / 2, 0, BlinkTask4, false, 2 * count);
			}
			else{
				App::mApp->mTask->Attach(period / 2, 0, BlinkTask4, true);
			}
		}
		else{
			App::mApp->mTask->DeAttach(BlinkTask4);
		}
	}
}
