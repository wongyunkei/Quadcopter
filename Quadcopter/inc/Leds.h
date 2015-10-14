/*
 * Leds.h
 *
 *  Created on: 2014�~8��3��
 *      Author: YunKei
 */

#ifndef LEDS_H_
#define LEDS_H_

#include <inttypes.h>
#include <stm32f4xx.h>

namespace Debug{

	class Leds{

		public:

			Leds();
			static Leds* getInstance();
			enum LEDS {LED1, LED2, LED3, LED4};
			void LedsControl(LEDS led, bool onState);
			void Toggle(LEDS led);
			void Blink(uint16_t period, LEDS leds, bool onState, int count = -1);
			GPIO_TypeDef* getLedsGPIO();

		private:
			GPIO_TypeDef* Leds_GPIO;


	};
};

using namespace Debug;

#endif /* LEDS_H_ */
