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

		public:
			typedef void (*pTask)();
			Task();
			void Attach(float period, float phaseShift, pTask fn, char* fnName, bool isPeriodic, int BreakCout = -1, bool keepLoopping = true);
			void DeAttach(pTask);
			void Run(bool isPrintTaskNum = false);
			void resetBreakCount(pTask fn);
			void printDeration(int index);
			bool IsPrintTaskNum;
			int currentTaskNum;
			int hangCount;
			int Count;

			static uint16_t maxTaskNum;
			uint16_t TasksNum;
			int** duration;
			pTask* mTask;
			float* TaskPeriod;
			char** TaskName;

		private:
			float* PhaseShift;
			bool* IsPeriodic;
			int* _BreakCout;
			bool KeepLoopping;
			bool OnWatchDog;
			Ticks* mTicks;
	};
};

#endif /* TASK_H_ */
