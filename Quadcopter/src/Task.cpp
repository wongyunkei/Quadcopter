/*
 * Task.cpp
 *
 *  Created on: 2014¦~8¤ë6¤é
 *      Author: YunKei
 */

#include <Task.h>
#include <Ticks.h>
#include <stdio.h>
#include <Usart.h>
#include <stdio.h>
#include <stm32f4xx_iwdg.h>
#include <Communicating.h>
#include <Leds.h.bak>

Task* _mTask;

Task::Task(bool onWatchDog) : pTicks(0), OnWatchDog(onWatchDog), TasksNum(0), hangCount(0), currentTaskNum(0), IsPrintTaskNum(false), KeepLoopping(true), Count(-1){
	pTicks = new Ticks(onWatchDog);
	_mTask = this;
}

Task* Task::getInstance(){
	return _mTask;
}

void Task::resetBreakCount(pTask fn){
	for(int i = 0; i < TasksNum; i++){
		if(mTask[TasksNum] == fn){
			_BreakCout[i] = -1;
		}
	}
}

void Task::Attach(float period, float phaseShift, pTask fn, bool isPeriodic, int BreakCout, bool keepLoopping){

	if(TasksNum == MAX_TASKS_NUM){
		printf("\nCannot attach any more tasks!\n");
		return;
	}

	mTask[TasksNum] = fn;

	TaskPeriod[TasksNum] = period;
	PhaseShift[TasksNum] = phaseShift;
	IsPeriodic[TasksNum] = isPeriodic;
	_BreakCout[TasksNum] = BreakCout;

	TasksNum++;
	KeepLoopping = keepLoopping;
}

void Task::DeAttach(pTask fn){

	for(int i = 0; i < TasksNum; i++){

		if(mTask[i] == fn){

			for(int j = i; j < TasksNum - 1; j++){

				mTask[j] = mTask[j + 1];
				TaskPeriod[j] = TaskPeriod[j + 1];
				PhaseShift[j] = PhaseShift[j + 1];
				_BreakCout[j] = _BreakCout[j + 1];
				IsPeriodic[j] = IsPeriodic[j + 1];
			}
			TasksNum--;
			return;
		}
	}
}

void Task::printDeration(int index){
	printf("Task %d Duration: %d\n", index, duration[index][1] - duration[index][0]);
}

void Task::Run(bool isPrintTaskNum){

	IsPrintTaskNum = isPrintTaskNum;

	uint16_t ticksImg = 0;
	bool isBreak = false;

	do{
		if(pTicks->getTicks() != ticksImg){
			ticksImg = pTicks->getTicks();
			if(OnWatchDog){
				IWDG_ReloadCounter();
			}

			for(int i = 0; i < TasksNum; i++){
				if(pTicks->TicksComp(TaskPeriod[i], PhaseShift[i], ticksImg)){
					duration[i][0] = pTicks->getTicks();
					hangCount = 0;
					currentTaskNum = i;
					if(_BreakCout[i] != 0){
						mTask[i]();
						duration[i][1] = pTicks->getTicks();
						if(!IsPeriodic[i]){
							if(_BreakCout[i] > 0){
								_BreakCout[i]--;
							}
						}
					}
				}
			}

			for(int i = 0; i < TasksNum; i++){
				if(!IsPeriodic[i]){
					if(_BreakCout[i] == 0){
						DeAttach(mTask[i]);
						if(!KeepLoopping){
							isBreak = true;
						}
					}
				}
			}
		}

	}while(KeepLoopping || !isBreak);
}
