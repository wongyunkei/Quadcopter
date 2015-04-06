/*
 * Task.cpp
 *
 *  Created on: 2014¦~8¤ë6¤é
 *      Author: YunKei
 */

#include <Task.h>
#include <Ticks.h>
#include <stdio.h>
#include <Leds.h>
#include <Usart.h>
#include <stdio.h>
#include <stm32f4xx_iwdg.h>

Task* _mTask;

Task::Task() : TasksNum(0), _BreakCout(-1){
	pTicks = new Ticks();
	_mTask = this;
}

Task* Task::getInstance(){
	return _mTask;
}

void Task::resetBreakCount(){
	_BreakCout = -1;
}

void Task::Attach(uint16_t period, uint16_t phaseShift, pTask fn, bool isPeriodic, int BreakCout){

	if(TasksNum == MAX_TASKS_NUM){
		printf("\nCannot attach any more tasks!\n");
		return;
	}

	mTask[TasksNum] = fn;

	TaskPeriod[TasksNum] = period;
	PhaseShift[TasksNum] = phaseShift;
	IsPeriodic[TasksNum] = isPeriodic;

	TasksNum++;
	_BreakCout = BreakCout;
}

void Task::DeAttach(pTask fn){

	for(int i = 0; i < TasksNum; i++){

		if(mTask[i] == fn){

			for(int j = 0; j < TasksNum - i; j++){

				mTask[i] = mTask[i + 1];
				TaskPeriod[i] = TaskPeriod[i + 1];
				PhaseShift[i] = PhaseShift[i + 1];
			}
			TasksNum--;
			return;
		}
	}
}

void Task::Run(){

	uint16_t ticksImg = 0;

	while(_BreakCout < 0 || _BreakCout > 0 ){

		if(pTicks->getTicks() != ticksImg){

			ticksImg = pTicks->getTicks();
			IWDG_ReloadCounter();

			for(int i = 0; i < TasksNum; i++){

				if(pTicks->TicksComp(TaskPeriod[i], PhaseShift[i], ticksImg)){
					mTask[i]();
					if(!IsPeriodic[i]){
						if(_BreakCout == 1){
							DeAttach(mTask[i]);
						}
						if(_BreakCout > 0){
							_BreakCout--;
						}
					}
				}
			}
		}
	}
}

