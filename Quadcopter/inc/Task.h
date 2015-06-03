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

			Task();
			static Task* getInstance();
			void Attach(uint16_t, uint16_t, pTask, bool, int);
			void DeAttach(pTask);
			void Run(bool isPrintTaskNum= false);
			void resetBreakCount();
			void printDeration(int index);
			bool IsPrintTaskNum;
			int currentTaskNum;
			int hangCount;
			int Count;

			uint16_t TasksNum;
			int duration[MAX_TASKS_NUM][2];
			pTask mTask[MAX_TASKS_NUM];

		private:

			Ticks* pTicks;
			uint16_t TaskPeriod[MAX_TASKS_NUM];
			uint16_t PhaseShift[MAX_TASKS_NUM];
			uint16_t IsPeriodic[MAX_TASKS_NUM];
			int _BreakCout;
	};
};

using namespace Time;

#endif /* TASK_H_ */
