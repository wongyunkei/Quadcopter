/*
 * Config.cpp
 *
 *  Created on: 2015�~11��27��
 *      Author: wongy
 */

#include <Config.h>

Config::Config(){

	Encoder1Conf1 = new Encoder::EncoderConfiguration(new Configuration(RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_9),
													  new Configuration(RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_11), Encoder::EncoderConfiguration::TimerConf1, 42000000);

	Encoder2Conf1 = new Encoder::EncoderConfiguration(new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_6),
													  new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_7), Encoder::EncoderConfiguration::TimerConf2, 42000000);

	Encoder3Conf1 = new Encoder::EncoderConfiguration(new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_4),
														  new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_5), Encoder::EncoderConfiguration::TimerConf4, 42000000);

	LedConf1 = new Led::LedConfiguration(new Configuration(RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_12), Bit_SET);
	LedConf2 = new Led::LedConfiguration(new Configuration(RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_13), Bit_SET);
	LedConf3 = new Led::LedConfiguration(new Configuration(RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_14), Bit_SET);
	LedConf4 = new Led::LedConfiguration(new Configuration(RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_15), Bit_SET);

	GPIOConf1 = new Led::LedConfiguration(new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_0), Bit_RESET);

	UART1Conf1 = new UART::UARTConfiguration(USART1, 115200, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_6), GPIO_PinSource6, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7), GPIO_PinSource7, false);
	UART1Conf2 = new UART::UARTConfiguration(USART1, 115200, new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_9), GPIO_PinSource9, new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_10), GPIO_PinSource10, true);
	UART3Conf1 = new UART::UARTConfiguration(USART3, 115200, new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_10), GPIO_PinSource10, new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_11), GPIO_PinSource11);
	UART4Conf1 = new UART::UARTConfiguration(UART4, 115200, new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_0), GPIO_PinSource0, new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_1), GPIO_PinSource1);
	UART4Conf2 = new UART::UARTConfiguration(UART4, 115200, new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_10), GPIO_PinSource10, new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_11), GPIO_PinSource11);
	UART5Conf1 = new UART::UARTConfiguration(UART5, 115200, new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_12), GPIO_PinSource12, new Configuration(RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_2), GPIO_PinSource2);

	mPWMConf1 = new PWM::PWMConfiguration(new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_6), GPIO_PinSource6,
										  new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_7), GPIO_PinSource7,
										  new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_8), GPIO_PinSource8,
										  new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_9), GPIO_PinSource9,
										  40000);

	ADCConf1 = new ADConverter::ADCConfiguration(new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_3), ADC_Channel_3, ADC_SampleTime_480Cycles);

	Configuration** CS = new Configuration*[2];
	CS[0] = new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_15);
	CS[1] = new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_8);

	Spi1Conf1 = new Spi::SpiConfiguration(Spi::SpiConfiguration::SpiConf1, Spi::SpiConfiguration::PRESCALER8, Spi::SpiConfiguration::SPIMODE0,
										  new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_3),
										  new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_6),
										  new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_7), CS, false, 1);

	Spi1Conf2 = new Spi::SpiConfiguration(Spi::SpiConfiguration::SpiConf1, Spi::SpiConfiguration::PRESCALER8, Spi::SpiConfiguration::SPIMODE0,
										  new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_3),
										  new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_6),
										  new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_7), CS, true, 1);

	I2C1Conf1 = new I2C::I2CConfiguration(I2C1, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_6), GPIO_PinSource6, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7), GPIO_PinSource7, I2C::I2CConfiguration::SPEED_400K);
	I2C1Conf2 = new I2C::I2CConfiguration(I2C1, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_8), GPIO_PinSource8, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_9), GPIO_PinSource9, I2C::I2CConfiguration::SPEED_400K);
	I2C2Conf1 = new I2C::I2CConfiguration(I2C2, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_10), GPIO_PinSource10, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_11), GPIO_PinSource11, I2C::I2CConfiguration::SPEED_400K);
	I2C2Conf2 = new I2C::I2CConfiguration(I2C2, new Configuration(RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_1), GPIO_PinSource1, new Configuration(RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_0), GPIO_PinSource0, I2C::I2CConfiguration::SPEED_400K);

	SonicConf1 = new Sonic::SonicConfiguration(new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_0), new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_8), GPIO_PinSource8);
	SonicConf2 = new Sonic::SonicConfiguration(new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_1), new Configuration(RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_11), GPIO_PinSource11);
	SonicConf3 = new Sonic::SonicConfiguration(new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_2), new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_10), GPIO_PinSource10);
	SonicConf4 = new Sonic::SonicConfiguration(new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_3), new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_11), GPIO_PinSource11);
}
