/*
 * Task.h
 *
 *  Created on: 2014¦~8¤ë6¤é
 *      Author: YunKei
 */

#ifndef TASK_H_
#define TASK_H_

#include <inttypes.h>
#include <Ticks.h>

namespace Time{

	class Task{

		#define MAX_TASKS_NUM	256
		typedef void (*pTask)(void);

		public:

			Task(bool onWatchDog);
			static Task* getInstance();
			void Attach(float period, float phaseShift, pTask fn, bool isPeriodic, int BreakCout = -1, bool keepLoopping = true);
//			void Attach(pTask fn, bool isPeriodic, int BreakCout = -1, bool keepLoopping = true);
			void DeAttach(pTask);
			void Run(bool isPrintTaskNum = false);
			void resetBreakCount(pTask fn);
			void printDeration(int index);
			bool IsPrintTaskNum;
			int currentTaskNum;
			int hangCount;
			int Count;

			uint16_t TasksNum;
//			uint16_t mInstantaneousTasksNum;
			int duration[MAX_TASKS_NUM][2];
			pTask mTask[MAX_TASKS_NUM];
//			pTask mInstantaneousTask[MAX_TASKS_NUM];

		private:

			Ticks* pTicks;

//			uint16_t InstantaneousTaskPeriod[MAX_TASKS_NUM];
//			uint16_t InstantaneousPhaseShift[MAX_TASKS_NUM];
//			uint16_t InstantaneousIsPeriodic[MAX_TASKS_NUM];
//			int _InstantaneousBreakCout[MAX_TASKS_NUM];

			float TaskPeriod[MAX_TASKS_NUM];
			float PhaseShift[MAX_TASKS_NUM];
			bool IsPeriodic[MAX_TASKS_NUM];
			int _BreakCout[MAX_TASKS_NUM];
			bool KeepLoopping;
			bool OnWatchDog;
	};
};

using namespace Time;

#endif /* TASK_H_ */
