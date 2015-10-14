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

void Task::Attach(float period, float phaseShift, pTask fn, bool isPeriodic, int BreakCout, bool keepLoopping, bool isInstantaneous){
//	if(isInstantaneous){
//		if(mInstantaneousTasksNum == MAX_TASKS_NUM){
//			printf("\nCannot attach any more instantaneous tasks!\n");
//			return;
//		}
//
//		mInstantaneousTask[TasksNum] = fn;
//
//		InstantaneousTaskPeriod[TasksNum] = period;
//		InstantaneousPhaseShift[TasksNum] = phaseShift;
//		InstantaneousIsPeriodic[TasksNum] = isPeriodic;
//		_InstantaneousBreakCout[TasksNum] = BreakCout;
//
//		mInstantaneousTasksNum++;
//	}
//	else{
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
//	}
	KeepLoopping = keepLoopping;
}

//void Task::Attach(pTask fn, bool isPeriodic, int BreakCout, bool keepLoopping){
//	Attach(0, 0, fn, isPeriodic, BreakCout, keepLoopping, true);
//}

void Task::DeAttach(pTask fn, bool isInstantaneous){

//	if(isInstantaneous){
//		for(int i = 0; i < mInstantaneousTasksNum; i++){
//
//			if(mInstantaneousTask[i] == fn){
//
//				for(int j = 0; j < mInstantaneousTasksNum - i - 1; j++){
//
//					mInstantaneousTask[i] = mInstantaneousTask[i + 1];
//					InstantaneousTaskPeriod[i] = InstantaneousTaskPeriod[i + 1];
//					InstantaneousPhaseShift[i] = InstantaneousPhaseShift[i + 1];
//					_InstantaneousBreakCout[i] = _InstantaneousBreakCout[i + 1];
//				}
//				mInstantaneousTasksNum--;
//				return;
//			}
//		}
//	}
//	else{
		for(int i = 0; i < TasksNum; i++){

			if(mTask[i] == fn){

				for(int j = 0; j < TasksNum - i - 1; j++){

					mTask[i] = mTask[i + 1];
					TaskPeriod[i] = TaskPeriod[i + 1];
					PhaseShift[i] = PhaseShift[i + 1];
					_BreakCout[i] = _BreakCout[i + 1];
				}
				TasksNum--;
				return;
			}
		}
//	}
}

void Task::printDeration(int index){
	printf("Task %d Duration: %d\n", index, duration[index][1] - duration[index][0]);
}

void Task::Run(bool isPrintTaskNum){

	IsPrintTaskNum = isPrintTaskNum;

	uint16_t ticksImg = 0;
	bool isBreak = false;

	do{
//		for(int i = 0; i < mInstantaneousTasksNum; i++){
//			if(_InstantaneousBreakCout[i] != 0){
//				mInstantaneousTask[i];
//				if(!InstantaneousIsPeriodic[i]){
//					if(_InstantaneousBreakCout[i] > 0){
//						_InstantaneousBreakCout[i]--;
//					}
//				}
//			}
//		}
//
//		for(int i = 0; i < mInstantaneousTasksNum; i++){
//			if(!InstantaneousIsPeriodic[i]){
//				if(_InstantaneousBreakCout[i] == 0){
//					DeAttach(mInstantaneousTask[i], true);
//					if(!KeepLoopping){
//						isBreak = true;
//					}
//				}
//			}
//		}

		if(pTicks->getTicks() != ticksImg){
			ticksImg = pTicks->getTicks();
			if(OnWatchDog){
				IWDG_ReloadCounter();
			}

			for(int i = 0; i < TasksNum; i++){
				if(pTicks->TicksComp(TaskPeriod[i] * 10, PhaseShift[i] * 10, ticksImg)){
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
