/*
 * Ticks.cpp
 *
 *  Created on: 2014�~8��3��
 *      Author: YunKei
 */

#include <App.h>
#include <Ticks.h>
#include <stm32f4xx.h>
#include <stm32f4xx_iwdg.h>
#include <stdio.h>
#include <Task.h>
#include <Communicating.h>
#include <Controlling.h>
#include <Delay.h>

uint16_t Ticks::maxTicks = 10000;

void SysTick_Handler(void){
	App::mApp->mTicks->TicksIncrement();
	if(App::mApp->mTicks->getTicks() >= Ticks::maxTicks){
		App::mApp->mTicks->setTicks(0);
	}
//	if(App::mApp->mTask->IsPrintTaskNum){
//		if(App::mApp->mTask->hangCount++ > 10000){
//
//			App::mApp->mCommunicating1->Send(5,App::mApp->mTask->currentTaskNum);
//
//			while(App::mApp->mCommunicating1->getTxBufferCount() >= 4){
//				App::mApp->mCommunicating1->SendPoll();
//				Delay::DelayMS(10);
//			}
//		}
//	}
}

uint16_t Ticks::getTimeout(){

	if(getTicks() >= timeoutStartTimestamp){
		return getTicks() - timeoutStartTimestamp;
	}
	else{
		return maxTicks - timeoutStartTimestamp + getTicks();
	}
}

Ticks::Ticks(bool onWatchDog) : ticks(0), timeoutCount(0), timeoutStartTimestamp(0), OnWatchDog(onWatchDog){

	SysTick_Config(168000);
	if(onWatchDog){
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
		IWDG_SetPrescaler(IWDG_Prescaler_256);
		IWDG_SetReload(250);
		IWDG_ReloadCounter();
		IWDG_Enable();
	}
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
	return (timeoutCount - getTimeout() < 0) ? true : false;
}

void Ticks::setTimeout(uint16_t value){
	timeoutCount = value;
	timeoutStartTimestamp = getTicks();
}
