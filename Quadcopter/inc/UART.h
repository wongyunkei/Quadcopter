/*
 * UART.h
 *
 *  Created on: 2014¦~8¤ë4¤é
 *      Author: YunKei
 */

#ifndef UART_H_
#define UART_H_

#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <Configuration.h>

namespace Communication{

	class UART{

		public:
			class UARTConfiguration{
				public:
					UARTConfiguration(USART_TypeDef* UARTx, uint32_t baudrate, Configuration* tx, uint8_t txSource, Configuration* rx, uint8_t rxSource, bool UseDMA = false);
					USART_TypeDef* _UARTx;
					uint32_t _baudrate;
					Configuration* _tx;
					Configuration* _rx;
					uint8_t _txSource;
					uint8_t _rxSource;
					bool _UseDMA;
				private:
			};

			UART(UARTConfiguration* conf);
			int Read(char*, int);
			void setBufferCount(int);
			int getBufferCount();
			char* getBuffer();
			void setPrintUART();
			void Print(const char*, ...);
			bool getIsDmaBusy();
			void setIsDmaBusy(bool);
			char* getRxBuffer();
			void reset();
			uint32_t getBaudrate();

			UARTConfiguration* Conf;

		private:
			char Buffer[2048];
			char txBuffer[128];
			char rxBuffer[7];
			bool isDmaBusy;
			char* pBuffer;
			int BufferCount;
	};
};

using namespace Communication;

#endif /* UART_H_ */
