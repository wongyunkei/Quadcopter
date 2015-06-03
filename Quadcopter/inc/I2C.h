/*
 * I2C.h
 *
 *  Created on: 2014¦~8¤ë18¤é
 *      Author: YunKei
 */

#ifndef I2C_H_
#define I2C_H_

#include <inttypes.h>
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

namespace Communication{

	class I2C{

		public:

			enum CLOCK{SPEED_100K = 100000, SPEED_400K = 400000};
			I2C(I2C_TypeDef*, CLOCK, bool createdInstance = false);
			static I2C* getInstance(I2C_TypeDef*);
			bool Write(uint8_t, uint8_t, uint8_t);
			uint8_t Read(uint8_t, uint8_t);
			bool BurstWrite(uint8_t, uint8_t, uint8_t, uint8_t*);
			bool BurstRead(uint8_t, uint8_t, uint8_t, uint8_t*);
			int ErrorCount;
		private:
			bool CreatedInstance;
			I2C_TypeDef* _I2Cx;
			void ResetI2C();
	};
};

using namespace Communication;

#endif /* I2C_H_ */
