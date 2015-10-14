/*
 * Spi.cpp
 *
 *  Created on: 2014¦~8¤ë7¤é
 *      Author: YunKei
 */

#include <Spi.h>
#include <Delay.h>
#include <stdio.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <Ticks.h>
#include <Usart.h>
#include <Task.h>

#define CS_GPIO	GPIOD
#define CS_RCC	RCC_AHB1Periph_GPIOD
#define CS0_PIN	GPIO_Pin_0
#define CS1_PIN	GPIO_Pin_1
#define CS2_PIN	GPIO_Pin_2
#define CS3_PIN	GPIO_Pin_3
#define CS4_PIN	GPIO_Pin_4
#define CS5_PIN	GPIO_Pin_5

Spi* _mSpi1;
Spi* _mSpi2;

int spiDelayCount;

void resetSpi1Task(){
	if(spiDelayCount++ > 10){
		spiDelayCount = 0;
		Spi(SPI1, Spi::getInstance(SPI1)->getPrescaler(), Spi::getInstance(SPI1)->getSpiMode(), true);
		Task::getInstance()->DeAttach(resetSpi1Task);
	}
}

void resetSpi2Task(){
	if(spiDelayCount++ > 10){
		spiDelayCount = 0;
		Spi(SPI2, Spi::getInstance(SPI2)->getPrescaler(), Spi::getInstance(SPI2)->getSpiMode(), true);
		Task::getInstance()->DeAttach(resetSpi2Task);
	}
}

void Spi::resetSpi(){

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	spiDelayCount = 0;
	if(Spix == SPI1){
		//MISO
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		//MOSI
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		//SCK

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		//NSS
		GPIO_InitStructure.GPIO_Pin = CS0_PIN | CS1_PIN | CS2_PIN | CS3_PIN | CS4_PIN | CS5_PIN;
		GPIO_Init(CS_GPIO, &GPIO_InitStructure);
		Task::getInstance()->Attach(10, 0, resetSpi1Task, true);
	}
	else if(Spix == SPI2){

		//MISO & MOSI
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		//SCK
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		//NSS
		GPIO_InitStructure.GPIO_Pin = CS0_PIN | CS1_PIN | CS2_PIN | CS3_PIN | CS4_PIN | CS5_PIN;
		GPIO_Init(CS_GPIO, &GPIO_InitStructure);
		Task::getInstance()->Attach(10, 0, resetSpi2Task, true);
	}
}

Spi::Spi(SPI_TypeDef* spi, PRESCALER prescaler, SPIMODE spiMode, bool createdInstance) : _prescaler(prescaler), _spiMode(spiMode){

	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	uint16_t p;
	SPI_I2S_DeInit(spi);

	switch(prescaler)
	{
		case PRESCALER2:
			p = SPI_BaudRatePrescaler_2;
			break;
		case PRESCALER4:
			p = SPI_BaudRatePrescaler_4;
			break;
		case PRESCALER8:
			p = SPI_BaudRatePrescaler_8;
			break;
		case PRESCALER16:
			p = SPI_BaudRatePrescaler_16;
			break;
		case PRESCALER32:
			p = SPI_BaudRatePrescaler_32;
			break;
		case PRESCALER64:
			p = SPI_BaudRatePrescaler_64;
			break;
		case PRESCALER128:
			p = SPI_BaudRatePrescaler_128;
			break;
		case PRESCALER256:
			p = SPI_BaudRatePrescaler_256;
			break;
		default:
			break;

	}

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

	switch(spiMode){

		case SPIMODE0:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
			break;

		case SPIMODE1:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
			break;

		case SPIMODE2:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
			break;

		case SPIMODE3:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
			break;

		default:
			break;
	}

	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = p;

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	if (spi == SPI1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		//MISO
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
		//MOSI

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
		//SCK
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
		//NSS


		RCC_AHB1PeriphClockCmd(CS_RCC, ENABLE);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Pin = CS0_PIN | CS1_PIN | CS2_PIN | CS3_PIN | CS4_PIN | CS5_PIN;
		GPIO_Init(CS_GPIO, &GPIO_InitStructure);

		SPI_Init(SPI1, &SPI_InitStructure);
		SPI_Cmd(SPI1, ENABLE);
		if(!createdInstance){
			_mSpi1 = this;
		}
	}
	else if (spi == SPI2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		//MISO & MOSI
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI1);
		//SCK
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI1);
		//NSS

		RCC_AHB1PeriphClockCmd(CS_RCC, ENABLE);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Pin = CS0_PIN | CS1_PIN | CS2_PIN | CS3_PIN | CS4_PIN | CS5_PIN;
		GPIO_Init(CS_GPIO, &GPIO_InitStructure);

		//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//		GPIO_Init(GPIOB, &GPIO_InitStructure);
		for(int i = 0; i < 6; i++){
			ChipDeSelect(i);
		}
		SPI_Init(SPI2, &SPI_InitStructure);
		SPI_Cmd(SPI2, ENABLE);
		if(!createdInstance){
			_mSpi2 = this;
		}
	}
	Spix = spi;
}

Spi::PRESCALER Spi::getPrescaler(){
	return _prescaler;
}

Spi::SPIMODE Spi::getSpiMode(){
	return _spiMode;
}

Spi* Spi::getInstance(SPI_TypeDef* spi){
	Spi* pSpi;
	if(spi == SPI1){
		pSpi = _mSpi1;
	}
	else if(spi == SPI2){
		pSpi = _mSpi2;
	}
	return pSpi;
}

bool Spi::Byte(uint8_t byte, uint8_t* data){
	Ticks::getInstance()->setTimeout(3);
	while(SPI_I2S_GetFlagStatus(Spix, SPI_I2S_FLAG_TXE) == RESET){
		if(Ticks::getInstance()->Timeout()){
			return false;
		}
	}

	SPI_I2S_SendData(Spix, byte);
	Ticks::getInstance()->setTimeout(3);
	while(SPI_I2S_GetFlagStatus(Spix, SPI_I2S_FLAG_RXNE) == RESET){
		if(Ticks::getInstance()->Timeout()){
			return false;
		}
	}

	*data = (uint8_t)SPI_I2S_ReceiveData(Spix);
	return true;
}

int Spi::WriteRead(int index, uint8_t data){

	uint8_t value = 0;
	ChipSelect(index);
	Byte(data, &value);
	ChipDeSelect(index);
	return value;
}

bool Spi::WriteCmd(int index, uint8_t reg, uint8_t cmd){

	uint8_t v = 0;
	ChipSelect(index);
	if(!Byte(reg, &v)){
		ChipDeSelect(index);
		return false;
	}
	if(!Byte(cmd, &v)){
		ChipDeSelect(index);
		return false;
	}
	ChipDeSelect(index);
	return true;
}

bool Spi::ReadData(int index, uint8_t reg, uint8_t* value){

	uint8_t v = 0;
	ChipSelect(index);
	if(!Byte(reg, &v)){
		ChipDeSelect(index);
		return false;
	}

	if(!Byte(0x00, value)){
		ChipDeSelect(index);
		return false;
	}
	ChipDeSelect(index);
	return true;
}

bool Spi::WriteNBytes(int index, uint8_t reg, uint8_t length, uint8_t* pData){

	int i = 0;
	uint8_t v = 0;
	ChipSelect(index);
	if(!Byte(reg, &v)){
		ChipDeSelect(index);
		return false;
	}
	for(i = 0; i < length; i++){
		if(!Byte(*(pData + i), &v)){
			ChipDeSelect(index);
			return false;
		}
	}
	ChipDeSelect(index);
	return true;
}

bool Spi::ReadNBytes(int index, uint8_t reg, uint8_t length, uint8_t* pData){

	int i = 0;
	uint8_t v = 0;
	ChipSelect(index);
	if(!Byte(reg, &v)){
		ChipDeSelect(index);
		return false;
	}
	for(i = 0; i < length; i++){
		if(!Byte(0x00, (pData + i))){
			ChipDeSelect(index);
			return false;
		}
	}
	ChipDeSelect(index);
	return true;
}

void Spi::ChipSelect(int index){

	switch(index){
		case 0:
			GPIO_ResetBits(CS_GPIO, CS0_PIN);
			break;
		case 1:
			GPIO_ResetBits(CS_GPIO, CS1_PIN);
			break;
		case 2:
			GPIO_ResetBits(CS_GPIO, CS2_PIN);
			break;
		case 3:
			GPIO_ResetBits(CS_GPIO, CS3_PIN);
			break;
		case 4:
			GPIO_ResetBits(CS_GPIO, CS4_PIN);
			break;
		case 5:
			GPIO_ResetBits(CS_GPIO, CS5_PIN);
			break;
	}
//	if(Spix == SPI1){
//		GPIO_ResetBits(SPI1_GPIO, GPIO_Pin_15);
//	}
//	else{
//		GPIO_ResetBits(SPI2_GPIO, GPIO_Pin_12);
//	}
	Delay::DelayUS(1);
}

void Spi::ChipDeSelect(int index){

	switch(index){
		case 0:
			GPIO_SetBits(CS_GPIO, CS0_PIN);
			break;
		case 1:
			GPIO_SetBits(CS_GPIO, CS1_PIN);
			break;
		case 2:
			GPIO_SetBits(CS_GPIO, CS2_PIN);
			break;
		case 3:
			GPIO_SetBits(CS_GPIO, CS3_PIN);
			break;
		case 4:
			GPIO_SetBits(CS_GPIO, CS4_PIN);
			break;
		case 5:
			GPIO_SetBits(CS_GPIO, CS5_PIN);
			break;
	}

//	if(Spix == SPI1){
//
//		GPIO_SetBits(SPI1_GPIO, GPIO_Pin_15);
//	}
//	else{
//		GPIO_SetBits(SPI2_GPIO, GPIO_Pin_12);
//	}
	Delay::DelayUS(1);
}

//void Spi::_WriteNBytes(uint8_t reg, uint8_t length, uint8_t* pData){
//
//	int i = 0;
//	Byte(reg);
//	for(i = 0; i < length; i++){
//		if(Byte(0x00, (pData + i))){
//			ChipDeSelect(index);
//			return false;
//		}
//	}
//}
//
//void Spi::_ReadNBytes(uint8_t reg, uint8_t length, uint8_t* pData){
//
//	int i = 0;
//	Byte(reg);
//	for(i = 0; i < length; i++){
//		*(pData + i) = Byte(0x00);
//	}
//}
