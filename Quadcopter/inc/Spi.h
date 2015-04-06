/*
 * Spi.h
 *
 *  Created on: 2014¦~8¤ë7¤é
 *      Author: YunKei
 */

#ifndef SPI_H_
#define SPI_H_

#include <inttypes.h>
#include <stm32f4xx_spi.h>

namespace Communication{

	class Spi{

		public:

			enum PRESCALER{PRESCALER2, PRESCALER4, PRESCALER8, PRESCALER16, PRESCALER32, PRESCALER64, PRESCALER128, PRESCALER256};
			enum SPIMODE{SPIMODE0, SPIMODE1, SPIMODE2, SPIMODE3};
			Spi(SPI_TypeDef* spi, PRESCALER, SPIMODE, bool createdInstance = false);
			static Spi* getInstance(SPI_TypeDef* spi);
			PRESCALER getPrescaler();
			SPIMODE getSpiMode();
			uint8_t WriteRead(uint8_t);
			uint8_t WriteCmd(uint8_t, uint8_t);
			uint8_t ReadData(uint8_t);
			void WriteNBytes(uint8_t, uint8_t, uint8_t*);
			void ReadNBytes(uint8_t, uint8_t, uint8_t*);
			void _WriteNBytes(uint8_t, uint8_t, uint8_t*);
			void _ReadNBytes(uint8_t, uint8_t, uint8_t*);
			void resetSpi();

			void ChipSelect();
			void ChipDeSelect();
			uint8_t Byte( uint8_t);

		private:

			SPI_TypeDef* Spix;
			PRESCALER _prescaler;
			SPIMODE _spiMode;
	};
};

using namespace Communication;

#endif /* SPI_H_ */
