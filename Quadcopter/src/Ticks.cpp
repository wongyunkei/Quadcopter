/*
 * Ticks.cpp
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#include <Ticks.h>
#include <stm32f4xx.h>
#include <stm32f4xx_iwdg.h>
#include <stdio.h>

Ticks* _mTicks;

void SysTick_Handler(void){
	_mTicks->TicksIncrement();
	if(_mTicks->getTicks() >= MAX_TICKS){
		_mTicks->setTicks(0);
		if(_mTicks->getTimeout() > 0){
			_mTicks->setTimeout(_mTicks->getTimeout() - 1);
		}
	}
}

uint16_t Ticks::getTimeout(){

	if(getTicks() >= timeoutStartTimestamp){
		return getTicks() - timeoutStartTimestamp;
	}
	else{
		return MAX_TICKS - timeoutStartTimestamp + getTicks();
	}
}

Ticks::Ticks() : ticks(0), timeoutCount(0), timeoutStartTimestamp(0) {

	SysTick_Config(168000);
	_mTicks = this;
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	IWDG_SetReload(250);
	IWDG_ReloadCounter();
	IWDG_Enable();
}

Ticks* Ticks::getInstance(){
	return _mTicks;
}

bool Ticks::TicksComp(uint16_t period, uint16_t phaseShift, uint16_t ticksImg){

	if(ticksImg % period == phaseShift){
		return true;
	}
	else{
		return false;
	}
}

void Ticks::TicksIncrement(){

	ticks++;
}

void Ticks::setTicks(uint16_t setValue){

	ticks = setValue;
}

uint16_t Ticks::getTicks(){

	return ticks;
}

bool Ticks::Timeout(){
	return getTimeout() >= timeoutCount ? true : false;
}

void Ticks::setTimeout(uint16_t value){
	timeoutCount = value;
	timeoutStartTimestamp = getTicks();
}
