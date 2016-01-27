/*
 * Config.cpp
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#include <Config.h>

Config::Config(){

	LedConf1 = new Led::LedConfiguration(new Configuration(RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_12), Bit_RESET);
	LedConf2 = new Led::LedConfiguration(new Configuration(RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_13), Bit_RESET);
	LedConf3 = new Led::LedConfiguration(new Configuration(RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_14), Bit_RESET);
	LedConf4 = new Led::LedConfiguration(new Configuration(RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_15), Bit_RESET);

	GPIOConf1 = new Led::LedConfiguration(new Configuration(RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_4), Bit_SET);

	UART1Conf1 = new UART::UARTConfiguration(USART1, 115200, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_6), GPIO_PinSource6, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7), GPIO_PinSource7, true);
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

	I2C1Con1 = new I2C::I2CConfiguration(I2C1, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_6), GPIO_PinSource6, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7), GPIO_PinSource7, I2C::I2CConfiguration::SPEED_400K);
	I2C1Con2 = new I2C::I2CConfiguration(I2C1, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_8), GPIO_PinSource8, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_9), GPIO_PinSource9, I2C::I2CConfiguration::SPEED_400K);
	I2C2Con1 = new I2C::I2CConfiguration(I2C2, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_10), GPIO_PinSource10, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_11), GPIO_PinSource11, I2C::I2CConfiguration::SPEED_400K);
	I2C2Con2 = new I2C::I2CConfiguration(I2C2, new Configuration(RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_1), GPIO_PinSource1, new Configuration(RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_0), GPIO_PinSource0, I2C::I2CConfiguration::SPEED_400K);

	SonicConf1 = new Sonic::SonicConfiguration(new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_0), new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_8), GPIO_PinSource8);
	SonicConf2 = new Sonic::SonicConfiguration(new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_1), new Configuration(RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_11), GPIO_PinSource11);
	SonicConf3 = new Sonic::SonicConfiguration(new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_2), new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_10), GPIO_PinSource10);
	SonicConf4 = new Sonic::SonicConfiguration(new Configuration(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_3), new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_11), GPIO_PinSource11);
}
