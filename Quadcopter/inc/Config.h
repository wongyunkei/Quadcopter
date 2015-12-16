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
#include <PWM.h>
#include <ADConverter.h>
#include <I2C.h>
#include <Sonic.h>

using namespace Debug;
using namespace Communication;
using namespace Control;
using namespace Sensors;

namespace System{

	class Config{
		public:
			Config();
			Led::LedConfiguration* LedConf1;
			Led::LedConfiguration* LedConf2;
			Led::LedConfiguration* LedConf3;
			Led::LedConfiguration* LedConf4;
			Led::LedConfiguration* GPIOConf1;
			UART::UARTConfiguration* UART1Conf1;
			UART::UARTConfiguration* UART1Conf2;
			UART::UARTConfiguration* UART3Conf1;
			UART::UARTConfiguration* UART3Conf2;
			UART::UARTConfiguration* UART4Conf1;
			UART::UARTConfiguration* UART4Conf2;
			UART::UARTConfiguration* UART5Conf1;
			UART::UARTConfiguration* UART5Conf2;
			PWM::PWMConfiguration* mPWMConf1;
			ADConverter::ADCConfiguration* ADCConf1;
			I2C::I2CConfiguration* I2C1Con1;
			I2C::I2CConfiguration* I2C1Con2;
			I2C::I2CConfiguration* I2C2Con1;
			I2C::I2CConfiguration* I2C2Con2;
			Sonic::SonicConfiguration* SonicConf1;
			Sonic::SonicConfiguration* SonicConf2;
			Sonic::SonicConfiguration* SonicConf3;
			Sonic::SonicConfiguration* SonicConf4;
		private:
	};

};

#endif /* CONFIG_H_ */
