/*
 * Spi.cpp
 *
 *  Created on: 2014�~8��7��
 *      Author: YunKei
 */

#include <App.h>
#include <Spi.h>
#include <Delay.h>
#include <stdio.h>
#include <stdarg.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <Ticks.h>
#include <Task.h>
#include <UART.h>
#include <stm32f4xx_it.h>
#include <AdditionalTools.h>

using namespace Communication;
using namespace Utility;

void SPI1_IRQHandler()
{
	App::mApp->mTicks->setTimeout(3);
	if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET){
		App::mApp->mSpi1->Buffer[App::mApp->mSpi1->BufferCount++] = (char)(0x00ff & SPI_I2S_ReceiveData(SPI1));
		App::mApp->mSpi1->AvailableLength++;
		if(App::mApp->mSpi1->BufferCount == 2047){
			App::mApp->mSpi1->BufferCount = 0;
		}
		if(App::mApp->mSpi1->Conf->IsSlave){
			if(App::mApp->mSpi1->SlaveTxLength > 0){
				App::mApp->mTicks->setTimeout(3);
				while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){
					if(App::mApp->mTicks->Timeout()){
						if(!App::mApp->mSpi1->Conf->IsSlave){
							App::mApp->mSpi1->Reset();
						}
						return;
					}
				}
				SPI_I2S_SendData(SPI1, App::mApp->mSpi1->SlaveTxBuffer[App::mApp->mSpi1->SlaveTxBufferCount++]);
				App::mApp->mSpi1->SlaveTxLength--;
				if(App::mApp->mSpi1->SlaveTxBufferCount == 2047){
					App::mApp->mSpi1->SlaveTxBufferCount = 0;
				}
				App::mApp->mTicks->setTimeout(3);
				while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){
					if(App::mApp->mTicks->Timeout()){
						if(!App::mApp->mSpi1->Conf->IsSlave){
							App::mApp->mSpi1->Reset();
						}
						return;
					}
				}
			}
		}
	}

}

void Spi::Reset(){
	App::mApp->mLed3->LedControl(true);
	for(int i = 0; i < Conf->NumOfDevices; i++){
		ChipDeSelect(i);
		ChipSelect(i);
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(Conf->SCLK->_rcc, ENABLE);
		GPIO_InitStructure.GPIO_Pin = Conf->SCLK->_pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
		GPIO_Init(Conf->SCLK->_port, &GPIO_InitStructure);
		GPIO_ResetBits(Conf->SCLK->_port, Conf->SCLK->_pin);
		Delay::DelayUS(1);
		GPIO_SetBits(Conf->SCLK->_port, Conf->SCLK->_pin);
		Delay::DelayUS(1);
		GPIO_ResetBits(Conf->SCLK->_port, Conf->SCLK->_pin);
		ChipDeSelect(i);
	}
	Initialize(Conf);
}

void Spi::setSlaveTxBuffer(char* data, int length){
	if(SlaveTxBufferCount + SlaveTxLength > 2047){
		SlaveTxBufferCount = 0;
	}
	pSlaveTxBuffer = &SlaveTxBuffer[SlaveTxBufferCount + SlaveTxLength];
	for(int i = 0; i < length; i++){
		if(pSlaveTxBuffer >= SlaveTxBuffer + 2047){
			pSlaveTxBuffer = SlaveTxBuffer;
		}
		App::mApp->mSpi1->pSlaveTxBuffer[i] = *(data++);
	}
	SlaveTxLength += length;
}

Spi::SpiConfiguration::SpiConfiguration(SPIConfx spiConfx,
		PRESCALER prescaler,
		SPIMODE spimode,
		Configuration* sclk,
		Configuration* miso,
		Configuration* mosi,
		Configuration** cs,
		bool isSlave,
		int numOfDevices) : SpiConfx(spiConfx),
				Prescaler(prescaler),
				Spimode(spimode),
				SCLK(sclk),
				MOSI(mosi),
				MISO(miso),
				CS(cs),
				IsSlave(isSlave),
				NumOfDevices(numOfDevices){
}

Spi::Spi(SpiConfiguration* conf) : Conf(conf), Spix(0), BufferCount(0), pBuffer(Buffer), AvailableLength(0), pSlaveTxBuffer(SlaveTxBuffer), SlaveTxLength(0), SlaveTxBufferCount(0){
	Initialize(conf);
}

void Spi::Initialize(SpiConfiguration* conf){
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t prescaler;

	if(conf->SpiConfx == SpiConfiguration::SpiConf1){
		Spix = SPI1;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	}
	else if(conf->SpiConfx == SpiConfiguration::SpiConf2){
		Spix = SPI2;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	}

	SPI_I2S_DeInit(Spix);
	switch(conf->Prescaler)
	{
		case SpiConfiguration::PRESCALER2:
			prescaler = SPI_BaudRatePrescaler_2;
			break;
		case SpiConfiguration::PRESCALER4:
			prescaler = SPI_BaudRatePrescaler_4;
			break;
		case SpiConfiguration::PRESCALER8:
			prescaler = SPI_BaudRatePrescaler_8;
			break;
		case SpiConfiguration::PRESCALER16:
			prescaler = SPI_BaudRatePrescaler_16;
			break;
		case SpiConfiguration::PRESCALER32:
			prescaler = SPI_BaudRatePrescaler_32;
			break;
		case SpiConfiguration::PRESCALER64:
			prescaler = SPI_BaudRatePrescaler_64;
			break;
		case SpiConfiguration::PRESCALER128:
			prescaler = SPI_BaudRatePrescaler_128;
			break;
		case SpiConfiguration::PRESCALER256:
			prescaler = SPI_BaudRatePrescaler_256;
			break;
		default:
			break;
	}

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	if(conf->IsSlave){
		SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
		RCC_AHB1PeriphClockCmd(conf->CS[0]->_rcc, ENABLE);
		GPIO_InitStructure.GPIO_Pin = conf->CS[0]->_pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(conf->CS[0]->_port, &GPIO_InitStructure);
		uint8_t csSource;
		for(int i = 0; i < 16; i++){
			if(conf->CS[0]->_pin == _BV(i)){
				csSource = i;
			}
		}
		GPIO_PinAFConfig(conf->CS[0]->_port, csSource, GPIO_AF_SPI1);
	}
	else{
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		for(int i = 0; i < conf->NumOfDevices; i++){
			RCC_AHB1PeriphClockCmd(conf->CS[i]->_rcc, ENABLE);
			GPIO_InitStructure.GPIO_Pin = conf->CS[i]->_pin;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_Init(conf->CS[i]->_port, &GPIO_InitStructure);
		}
	}

	switch(conf->Spimode){

		case SpiConfiguration::SPIMODE0:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
			break;

		case SpiConfiguration::SPIMODE1:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
			break;

		case SpiConfiguration::SPIMODE2:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
			break;

		case SpiConfiguration::SPIMODE3:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
			break;

		default:
			break;
	}

	SPI_InitStructure.SPI_BaudRatePrescaler = prescaler;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	RCC_AHB1PeriphClockCmd(conf->SCLK->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->SCLK->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(conf->SCLK->_port, &GPIO_InitStructure);
	uint8_t clkSource;
	for(int i = 0; i < 16; i++){
		if(conf->SCLK->_pin == _BV(i)){
			clkSource = i;
		}
	}
	GPIO_PinAFConfig(conf->SCLK->_port, clkSource, GPIO_AF_SPI1);

	RCC_AHB1PeriphClockCmd(conf->MISO->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->MISO->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(conf->MISO->_port, &GPIO_InitStructure);
	uint8_t misoSource;
	for(int i = 0; i < 16; i++){
		if(conf->MISO->_pin == _BV(i)){
			misoSource = i;
		}
	}
	GPIO_PinAFConfig(conf->MISO->_port, misoSource, GPIO_AF_SPI1);


	RCC_AHB1PeriphClockCmd(conf->MOSI->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->MOSI->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(conf->MOSI->_port, &GPIO_InitStructure);
	uint8_t mosiSource;
	for(int i = 0; i < 16; i++){
		if(conf->MOSI->_pin == _BV(i)){
			mosiSource = i;
		}
	}
	GPIO_PinAFConfig(conf->MOSI->_port, mosiSource, GPIO_AF_SPI1);


	SPI_I2S_ITConfig(Spix, SPI_I2S_IT_RXNE, ENABLE);
	SPI_Init(Spix, &SPI_InitStructure);
	SPI_Cmd(Spix, ENABLE);
}

bool Spi::SendByte(uint8_t byte){

	App::mApp->mTicks->setTimeout(3);
	while(SPI_I2S_GetFlagStatus(Spix, SPI_I2S_FLAG_TXE) == RESET){
		if(App::mApp->mTicks->Timeout()){
			return false;
		}
	}
	SPI_I2S_SendData(Spix, byte);
	App::mApp->mTicks->setTimeout(3);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){
		if(App::mApp->mTicks->Timeout()){
			return false;
		}
	}
	return true;
}

bool Spi::Byte(uint8_t byte, uint8_t* data){
	App::mApp->mTicks->setTimeout(3);
	while(SPI_I2S_GetFlagStatus(Spix, SPI_I2S_FLAG_TXE) == RESET){
		if(App::mApp->mTicks->Timeout()){
			return false;
		}
	}

	SPI_I2S_SendData(Spix, byte);
	App::mApp->mTicks->setTimeout(3);
	while(SPI_I2S_GetFlagStatus(Spix, SPI_I2S_FLAG_RXNE) == RESET){
		if(App::mApp->mTicks->Timeout()){
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
		if(!Conf->IsSlave){
			Reset();
		}
		return false;
	}
	if(!Byte(cmd, &v)){
		if(!Conf->IsSlave){
			Reset();
		}
		return false;
	}
	ChipDeSelect(index);
	return true;
}

void Spi::Print(int index, const char* pstr, ...)
{
	int length = 0;
	va_list arglist;
	char* fp;
	for(int i = 0; i < 128; i++){
		txBuffer[i] = 0;
	}
	va_start(arglist, pstr);
	vsprintf(txBuffer, pstr, arglist);
	va_end(arglist);

	fp = txBuffer;

	while(*(fp++)){
		length++;
	}
	for(int i = 0; i < length; i++){
		Transfer(index, txBuffer[i]);
	}
}


bool Spi::Transfer(int index, uint8_t data){

	ChipSelect(index);
	if(!SendByte(data)){
		if(!Conf->IsSlave){
			Reset();
		}
		return false;
	}
	ChipDeSelect(index);
	return true;
}

int Spi::SlaveTransfer(){

	uint8_t value = 0;
	if(!Byte(0, &value)){
		return 0;
	}
	return value;
}

bool Spi::ReadData(int index, uint8_t reg, uint8_t* value){

	uint8_t v = 0;
	ChipSelect(index);
	if(!Byte(reg, &v)){
		if(!Conf->IsSlave){
			Reset();
		}
		return false;
	}

	if(!Byte(0x00, value)){
		if(!Conf->IsSlave){
			Reset();
		}
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
		if(!Conf->IsSlave){
			Reset();
		}
		return false;
	}
	for(i = 0; i < length; i++){
		if(!Byte(*(pData + i), &v)){
			if(!Conf->IsSlave){
				Reset();
			}
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
		if(!Conf->IsSlave){
			Reset();
		}
		return false;
	}
	for(i = 0; i < length; i++){
		if(!Byte(0x00, (pData + i))){
			if(!Conf->IsSlave){
				Reset();
			}
			return false;
		}
	}
	ChipDeSelect(index);
	return true;
}

void Spi::ChipSelect(int index){
	GPIO_ResetBits(Conf->CS[index]->_port, Conf->CS[index]->_pin);
	Delay::DelayUS(1);
}

void Spi::ChipDeSelect(int index){
	GPIO_SetBits(Conf->CS[index]->_port, Conf->CS[index]->_pin);
	Delay::DelayUS(1);
}

int Spi::Read(char* buffer, int length){
	pBuffer = &Buffer[BufferCount - AvailableLength];
	for(int i = 0; i < length; i++){
		if(pBuffer >= Buffer + 2047){
			pBuffer = Buffer;
		}
		buffer[i] = *(pBuffer++);
	}
	buffer[length] = '\0';
	AvailableLength -= length;
	return AvailableLength;
}
