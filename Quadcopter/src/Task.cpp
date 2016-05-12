/*
 * Task.cpp
 *
 *  Created on: 2014�~8��6��
 *      Author: YunKei
 */

#include <App.h>
#include <Task.h>
#include <Ticks.h>
#include <stdio.h>
#include <stdio.h>
#include <stm32f4xx_iwdg.h>
#include <Communicating.h>
#include <UART.h>
#include <string>

uint16_t Task::maxTaskNum = 1024;

Task::Task() : mTicks(App::mApp->mTicks), OnWatchDog(App::mApp->mTicks->OnWatchDog), TasksNum(0), hangCount(0), currentTaskNum(0), IsPrintTaskNum(false), KeepLoopping(true), Count(-1){
	duration = new int*[maxTaskNum];
	for(int i = 0; i < maxTaskNum; i++){
		duration[i] = new int[2];
	}
	mTask = new pTask[maxTaskNum];
	TaskName = new char*[maxTaskNum];
	TaskPeriod = new float[maxTaskNum];
	PhaseShift = new float[maxTaskNum];
	IsPeriodic = new bool[maxTaskNum];
	_BreakCout = new int[maxTaskNum];
}

void Task::resetBreakCount(pTask fn){
	for(int i = 0; i < TasksNum; i++){
		if(mTask[TasksNum] == fn){
			_BreakCout[i] = -1;
		}
	}
}

void Task::Attach(float period, float phaseShift, pTask fn, char* fnName, bool isPeriodic, int BreakCout, bool keepLoopping){

	if(TasksNum == maxTaskNum){
		printf("\nCannot attach any more tasks!\n");
		return;
	}

	mTask[TasksNum] = fn;

	TaskPeriod[TasksNum] = period;
	TaskName[TasksNum] = fnName;
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
		if(mTicks->getTicks() != ticksImg){
			ticksImg = mTicks->getTicks();
			if(OnWatchDog){
				IWDG_ReloadCounter();
			}

			for(int i = 0; i < TasksNum; i++){
				if(mTicks->TicksComp(TaskPeriod[i], PhaseShift[i], ticksImg)){
					duration[i][0] = mTicks->getTicks();
					hangCount = 0;
					currentTaskNum = i;
					if(_BreakCout[i] != 0){
						mTask[i]();
						duration[i][1] = mTicks->getTicks();
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
