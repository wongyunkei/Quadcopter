/*
 * Config.h
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <Configuration.h>
#include <Led.h>
#include <UART.h>
#include <Communicating.h>
#include <Sonic.h>

namespace System{

	class Config{
		public:
			Config();
			Led::LedConfiguration* LedConf1;
			Led::LedConfiguration* LedConf2;
			Led::LedConfiguration* LedConf3;
			Led::LedConfiguration* LedConf4;
			UART::UARTConfiguration* UART1Conf1;
			UART::UARTConfiguration* UART1Conf2;
			UART::UARTConfiguration* UART3Conf1;
			UART::UARTConfiguration* UART3Conf2;
			UART::UARTConfiguration* UART4Conf1;
			UART::UARTConfiguration* UART4Conf2;
			UART::UARTConfiguration* UART5Conf1;
			UART::UARTConfiguration* UART5Conf2;
			Sonic::SonicConfiguration* SonicConf1;
			Sonic::SonicConfiguration* SonicConf2;
			Sonic::SonicConfiguration* SonicConf3;
			Sonic::SonicConfiguration* SonicConf4;
		private:

	};

};

using namespace System;

#endif /* CONFIG_H_ */
