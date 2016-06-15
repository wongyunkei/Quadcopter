/*
 * Config.h
 *
 *  Created on: 2015�~11��27��
 *      Author: wongy
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <Configuration.h>
#include <GPIO.h>
#include <UART.h>
#include <Communicating.h>
#include <PWM.h>
#include <ADConverter.h>
#include <I2C.h>
#include <Spi.h>
#include <Sonic.h>
#include <Encoder.h>

using namespace System;
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
			GPIO::GPIOConfiguration* LedConf1;
			GPIO::GPIOConfiguration* LedConf2;
			GPIO::GPIOConfiguration* LedConf3;
			GPIO::GPIOConfiguration* LedConf4;
			GPIO::GPIOConfiguration* LedConf5;
			GPIO::GPIOConfiguration* LedConf6;
			GPIO::GPIOConfiguration* LedConf7;
			GPIO::GPIOConfiguration* LedConf8;
			GPIO::GPIOConfiguration* GPIOConf1;
			GPIO::GPIOConfiguration* GPIOConf2;
			GPIO::GPIOConfiguration* GPIOConf3;
			GPIO::GPIOConfiguration* GPIOConf4;
			GPIO::GPIOConfiguration* GPIOConf5;
			GPIO::GPIOConfiguration* GPIOConf6;
			GPIO::GPIOConfiguration* GPIOConf7;
			GPIO::GPIOConfiguration* GPIOConf8;
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
