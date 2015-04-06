/*
 * Spi.cpp
 *
 *  Created on: 2014¦~8¤ë7¤é
 *      Author: YunKei
 */

#include <Spi.h>
#include <Delay.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <Ticks.h>
#include <Usart.h>
#include <Task.h>

#define SPI1_GPIO	GPIOA
#define SPI2_GPIO	GPIOB

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
		//MISO & MOSI
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		//SCK
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		//NSS
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		Task::getInstance()->Attach(10, 0, resetSpi1Task, true, -1);
	}
	else if(Spix == SPI2){

		//MISO & MOSI
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		//SCK
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		//NSS
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		Task::getInstance()->Attach(10, 0, resetSpi2Task, true, -1);
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
		//MISO & MOSI
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
		//SCK
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
		//NSS
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

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
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

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

uint8_t Spi::Byte(uint8_t byte){
	Ticks::getInstance()->setTimeout(2);
	while(SPI_I2S_GetFlagStatus(Spix, SPI_I2S_FLAG_TXE) == RESET){
		if(Ticks::getInstance()->Timeout()){
			return 0;
		}
	}
	SPI_I2S_SendData(Spix, byte);
	Ticks::getInstance()->setTimeout(2);
	while(SPI_I2S_GetFlagStatus(Spix, SPI_I2S_FLAG_RXNE) == RESET){
		if(Ticks::getInstance()->Timeout()){
			return 0;
		}
	}
	return SPI_I2S_ReceiveData(Spix);
}

uint8_t Spi::WriteRead(uint8_t data){

	uint8_t value = 0;
	ChipSelect();
	value = Byte(data);
	ChipDeSelect();
	return value;
}

uint8_t Spi::WriteCmd(uint8_t reg, uint8_t cmd){

	uint8_t value = 0;
	ChipSelect();
	Byte(reg);
	value = Byte(cmd);
	ChipDeSelect();
	return value;
}

uint8_t Spi::ReadData(uint8_t reg){

	uint8_t value = 0;
	ChipSelect();
	Byte(reg);
	value = Byte(0x00);
	ChipDeSelect();
	return value;
}

void Spi::WriteNBytes(uint8_t reg, uint8_t length, uint8_t* pData){

	int i = 0;
	ChipSelect();
	Byte(reg);
	for(i = 0; i < length; i++){
		Byte(*(pData + i));
	}
	ChipDeSelect();
}

void Spi::ReadNBytes(uint8_t reg, uint8_t length, uint8_t* pData){

	int i = 0;
	ChipSelect();
	Byte(reg);
	for(i = 0; i < length; i++){
		*(pData + i) = Byte(0x00);
	}
	ChipDeSelect();
}

void Spi::ChipSelect(){

	if(Spix == SPI1){
		GPIO_ResetBits(SPI1_GPIO, GPIO_Pin_15);
	}
	else{
		GPIO_ResetBits(SPI2_GPIO, GPIO_Pin_12);
	}
	Delay::DelayUS(1);
}

void Spi::ChipDeSelect(){

	if(Spix == SPI1){

		GPIO_SetBits(SPI1_GPIO, GPIO_Pin_15);
	}
	else{
		GPIO_SetBits(SPI2_GPIO, GPIO_Pin_12);
	}
	Delay::DelayUS(1);
}

void Spi::_WriteNBytes(uint8_t reg, uint8_t length, uint8_t* pData){

	int i = 0;
	Byte(reg);
	for(i = 0; i < length; i++){
		Byte(*(pData + i));
	}
}

void Spi::_ReadNBytes(uint8_t reg, uint8_t length, uint8_t* pData){

	int i = 0;
	Byte(reg);
	for(i = 0; i < length; i++){
		*(pData + i) = Byte(0x00);
	}
}
