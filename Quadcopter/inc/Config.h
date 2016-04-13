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
#include <Spi.h>
#include <Sonic.h>
#include <Encoder.h>

using namespace Debug;
using namespace Communication;
using namespace Control;
using namespace Sensors;

namespace System{

	class Config{
		public:
			Config();
			Encoder::EncoderConfiguration* Encoder1Conf1;
			Encoder::EncoderConfiguration* Encoder2Conf1;
			Encoder::EncoderConfiguration* Encoder3Conf1;
			Encoder::EncoderConfiguration* Encoder4Conf1;
			Encoder::EncoderConfiguration* Encoder5Conf1;
			Encoder::EncoderConfiguration* Encoder6Conf1;
			Led::LedConfiguration* LedConf1;
			Led::LedConfiguration* LedConf2;
			Led::LedConfiguration* LedConf3;
			Led::LedConfiguration* LedConf4;
			Led::LedConfiguration* LedConf5;
			Led::LedConfiguration* LedConf6;
			Led::LedConfiguration* LedConf7;
			Led::LedConfiguration* LedConf8;
			Led::LedConfiguration* LedConf9;
			Led::LedConfiguration* LedConf10;
			Led::LedConfiguration* LedConf11;
			Led::LedConfiguration* LedConf12;
			Led::LedConfiguration* LedConf13;
			Led::LedConfiguration* LedConf14;
			Led::LedConfiguration* LedConf15;
			Led::LedConfiguration* LedConf16;
			Led::LedConfiguration* LedConf17;
			Led::LedConfiguration* LedConf18;
			Led::LedConfiguration* LedConf19;
			Led::LedConfiguration* LedConf20;
			Led::LedConfiguration* GPIOConf1;
			Led::LedConfiguration* GPIOConf2;
			Led::LedConfiguration* GPIOConf3;
			Led::LedConfiguration* GPIOConf4;
			Led::LedConfiguration* GPIOConf5;
			Led::LedConfiguration* GPIOConf6;
			Led::LedConfiguration* GPIOConf7;
			Led::LedConfiguration* GPIOConf8;
			UART::UARTConfiguration* UART1Conf1;
			UART::UARTConfiguration* UART1Conf2;
			UART::UARTConfiguration* UART2Conf1;
			UART::UARTConfiguration* UART2Conf2;
			UART::UARTConfiguration* UART3Conf1;
			UART::UARTConfiguration* UART3Conf2;
			UART::UARTConfiguration* UART4Conf1;
			UART::UARTConfiguration* UART4Conf2;
			UART::UARTConfiguration* UART5Conf1;
			UART::UARTConfiguration* UART5Conf2;
			PWM::PWMConfiguration* mPWMConf1;
			ADConverter::ADCConfiguration* ADCConf1;
			I2C::I2CConfiguration* I2C1Conf1;
			I2C::I2CConfiguration* I2C1Conf2;
			I2C::I2CConfiguration* I2C2Conf1;
			I2C::I2CConfiguration* I2C2Conf2;
			Spi::SpiConfiguration* Spi1Conf1;
			Spi::SpiConfiguration* Spi1Conf2;
			Spi::SpiConfiguration* Spi2Conf1;
			Sonic::SonicConfiguration* SonicConf1;
			Sonic::SonicConfiguration* SonicConf2;
			Sonic::SonicConfiguration* SonicConf3;
			Sonic::SonicConfiguration* SonicConf4;
		private:
	};

};

#endif /* CONFIG_H_ */
