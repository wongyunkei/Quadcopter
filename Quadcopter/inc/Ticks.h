/*
 * Ticks.h
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#ifndef TICKS_H_
#define TICKS_H_

#include <inttypes.h>
#include <stm32f4xx_it.h>

namespace Time{

	class Ticks{

		#define MAX_TICKS 10000

		public:

			Ticks(bool onWatchDog);
			static Ticks* getInstance();
			void TicksIncrement();
			bool TicksComp(uint16_t, uint16_t, uint16_t);
			void setTicks(uint16_t);
			void setMaxTicks(uint16_t);
			uint16_t getTicks();
			uint16_t getMaxTicks();
			void setTimeout(uint16_t);
			bool Timeout();
			uint16_t getTimeout();

		private:

			uint16_t ticks;
			uint16_t timeoutCount;
			uint16_t timeoutStartTimestamp;
	};
};

using namespace Time;

#endif /* TICKS_H_ */
