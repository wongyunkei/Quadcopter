/*
 * UART.cpp
 *
 *  Created on: 2014�~8��4��
 *      Author: YunKei
 */

#include <App.h>
#include <stm32f4xx_usart.h>
#include <stdio.h>
#include <stm32f4xx_it.h>
#include <Ticks.h>
#include <stdarg.h>
#include <stm32f4xx_dma.h>
#include <Delay.h>
#include <Task.h>
#include <UART.h>

extern USART_TypeDef* STDOUT_USART;
extern USART_TypeDef* STDERR_USART;
extern USART_TypeDef* STDIN_USART;

UART::UARTConfiguration::UARTConfiguration(USART_TypeDef* UARTx, uint32_t baudrate, Configuration* tx, uint8_t txSource, Configuration* rx, uint8_t rxSource, bool UseDMA) : _UARTx(UARTx), _baudrate(baudrate), _tx(tx), _rx(rx), _txSource(txSource), _rxSource(rxSource), _UseDMA(UseDMA){
};

void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) == SET)
	{
		DMA_ClearITPendingBit (DMA2_Stream7, DMA_IT_TCIF7);
		DMA_Cmd(DMA2_Stream7, DISABLE);
		App::mApp->mTicks->setTimeout(3);
		while (USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET){
			if(App::mApp->mTicks->Timeout()){
				break;
			}
		}
		USART_ClearFlag(USART1,USART_FLAG_TC);
		App::mApp->mUART1->isDmaBusy = false;//setIsDmaBusy(false);
	}
}

void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) == SET)
	{
		DMA_ClearITPendingBit (DMA1_Stream3, DMA_IT_TCIF3);
		DMA_Cmd(DMA1_Stream3, DISABLE);
		App::mApp->mTicks->setTimeout(3);
		while (USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET){
			if(App::mApp->mTicks->Timeout()){
				break;
			}
		}
		USART_ClearFlag(USART3,USART_FLAG_TC);
		App::mApp->mUART3->isDmaBusy = false;//setIsDmaBusy(false);
	}
}

void DMA1_Stream1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1) == SET)
	{
		DMA_ClearITPendingBit (DMA1_Stream1, DMA_IT_TCIF1);
		DMA_Cmd(DMA1_Stream1, DISABLE);
		App::mApp->mTicks->setTimeout(3);
		while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET){
			if(App::mApp->mTicks->Timeout()){
				break;
			}
		}
		for(int i = 0; i < 4; i++){
//			App::mApp->mUART3->getBuffer()[App::mApp->mUART3->getBufferCount()] = App::mApp->mUART3->getRxBuffer()[i];
//			App::mApp->mUART3->setBufferCount(App::mApp->mUART3->getBufferCount() + 1);
//			if(App::mApp->mUART3->getBufferCount() == 2047){
//				App::mApp->mUART3->setBufferCount(0);
//			}
			App::mApp->mUART3->Buffer[App::mApp->mUART3->BufferCount++] = App::mApp->mUART3->rxBuffer[i];
			App::mApp->mUART3->AvailableLength++;
			if(App::mApp->mUART3->BufferCount >= 2047){
				App::mApp->mUART3->BufferCount = 0;
			}
		}

		DMA_Cmd(DMA1_Stream1, ENABLE);
	}
}

void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) == SET)
	{
		DMA_ClearITPendingBit (DMA2_Stream2, DMA_IT_TCIF2);
		DMA_Cmd(DMA2_Stream2, DISABLE);
		App::mApp->mTicks->setTimeout(3);
		while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET){
			if(App::mApp->mTicks->Timeout()){
				break;
			}
		}
		for(int i = 0; i < 4; i++){
			App::mApp->mUART1->Buffer[App::mApp->mUART1->BufferCount++] = App::mApp->mUART1->rxBuffer[i];
			App::mApp->mUART1->AvailableLength++;
			if(App::mApp->mUART1->BufferCount >= 2047){
				App::mApp->mUART1->BufferCount = 0;
			}
		}
		DMA_Cmd(DMA2_Stream2, ENABLE);
	}
}


void USART1_IRQHandler(){
	App::mApp->mUART1->Buffer[App::mApp->mUART1->BufferCount++] = USART_ReceiveData(USART1);
	App::mApp->mUART1->AvailableLength++;
	if(App::mApp->mUART1->BufferCount >= 2047){
		App::mApp->mUART1->BufferCount = 0;
	}
}

void USART3_IRQHandler(){
	App::mApp->mUART3->Buffer[App::mApp->mUART3->BufferCount++] = USART_ReceiveData(USART3);
	App::mApp->mUART3->AvailableLength++;
	if(App::mApp->mUART3->BufferCount >= 2047){
		App::mApp->mUART3->BufferCount = 0;
	}
}

void UART4_IRQHandler(){
	App::mApp->mUART4->Buffer[App::mApp->mUART4->BufferCount++] = USART_ReceiveData(UART4);
	App::mApp->mUART4->AvailableLength++;
	if(App::mApp->mUART4->BufferCount >= 2047){
		App::mApp->mUART4->BufferCount = 0;
	}
}

void UART5_IRQHandler(){
	App::mApp->mUART5->Buffer[App::mApp->mUART5->BufferCount++] = USART_ReceiveData(UART5);
	App::mApp->mUART5->AvailableLength++;
	if(App::mApp->mUART5->BufferCount >= 2047){
		App::mApp->mUART5->BufferCount = 0;
	}
}

int UART::Read(char* buffer, int length){
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

void UART::setPrintUART(){
	STDOUT_USART = Conf->_UARTx;
	STDERR_USART = Conf->_UARTx;
	STDIN_USART = Conf->_UARTx;
}

UART::UART(UARTConfiguration* conf) : Conf(conf), BufferCount(0), pBuffer(Buffer), isDmaBusy(false), AvailableLength(0){

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	USART_InitStructure.USART_BaudRate = conf->_baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	if (conf->_UARTx == USART1)
	{
		USART_DeInit(USART1);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_AHB1PeriphClockCmd(conf->_tx->_rcc, ENABLE);
		GPIO_PinAFConfig(conf->_tx->_port, conf->_txSource, GPIO_AF_USART1);
		GPIO_InitStructure.GPIO_Pin = conf->_tx->_pin;
		GPIO_Init(conf->_tx->_port, &GPIO_InitStructure);

		RCC_AHB1PeriphClockCmd(conf->_rx->_rcc, ENABLE);
		GPIO_PinAFConfig(conf->_rx->_port, conf->_rxSource, GPIO_AF_USART1);
		GPIO_InitStructure.GPIO_Pin = conf->_rx->_pin;
		GPIO_Init(conf->_rx->_port, &GPIO_InitStructure);

		USART_Init(USART1, &USART_InitStructure);
		USART_Cmd(USART1, ENABLE);

		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		DMA_DeInit(DMA2_Stream7);

		DMA_InitStructure.DMA_BufferSize = 32;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART1->DR)) ;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(txBuffer) ;
		DMA_Init(DMA2_Stream7, &DMA_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init (&NVIC_InitStructure);

		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		DMA_ITConfig (DMA2_Stream7, DMA_IT_TC, ENABLE);


		DMA_DeInit(DMA2_Stream2);

		DMA_InitStructure.DMA_BufferSize = 4;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART1->DR)) ;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(rxBuffer) ;
		DMA_Init(DMA2_Stream2, &DMA_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init (&NVIC_InitStructure);

		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
		DMA_ITConfig (DMA2_Stream2, DMA_IT_TC, ENABLE);
		DMA_Cmd(DMA2_Stream2, ENABLE);
	}
	else if(conf->_UARTx == USART3){

		USART_DeInit(USART3);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

		RCC_AHB1PeriphClockCmd(conf->_tx->_rcc, ENABLE);
		GPIO_PinAFConfig(conf->_tx->_port, conf->_txSource, GPIO_AF_USART3);
		GPIO_InitStructure.GPIO_Pin = conf->_tx->_pin;
		GPIO_Init(conf->_tx->_port, &GPIO_InitStructure);

		RCC_AHB1PeriphClockCmd(conf->_rx->_rcc, ENABLE);
		GPIO_PinAFConfig(conf->_rx->_port, conf->_rxSource, GPIO_AF_USART3);
		GPIO_InitStructure.GPIO_Pin = conf->_rx->_pin;
		GPIO_Init(conf->_rx->_port, &GPIO_InitStructure);

		USART_Init(USART3, &USART_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;

		USART_Cmd(USART3, ENABLE);

		RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA1, ENABLE);
		DMA_DeInit(DMA1_Stream3);

		DMA_InitStructure.DMA_BufferSize = 32;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART3->DR)) ;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(txBuffer) ;
		DMA_Init(DMA1_Stream3, &DMA_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init (&NVIC_InitStructure);

		USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
		DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);


		DMA_DeInit(DMA1_Stream1);

		DMA_InitStructure.DMA_BufferSize = 4;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART3->DR)) ;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(rxBuffer) ;
		DMA_Init(DMA1_Stream1, &DMA_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init (&NVIC_InitStructure);

		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
		DMA_ITConfig (DMA1_Stream1, DMA_IT_TC, ENABLE);
		DMA_Cmd(DMA1_Stream1, ENABLE);

	}
	else if (conf->_UARTx == UART4)
	{
		USART_DeInit(UART4);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		RCC_AHB1PeriphClockCmd(conf->_tx->_rcc, ENABLE);
		GPIO_PinAFConfig(conf->_tx->_port, conf->_txSource, GPIO_AF_UART4);
		GPIO_InitStructure.GPIO_Pin = conf->_tx->_pin;
		GPIO_Init(conf->_tx->_port, &GPIO_InitStructure);

		RCC_AHB1PeriphClockCmd(conf->_rx->_rcc, ENABLE);
		GPIO_PinAFConfig(conf->_rx->_port, conf->_rxSource, GPIO_AF_UART4);
		GPIO_InitStructure.GPIO_Pin = conf->_rx->_pin;
		GPIO_Init(conf->_rx->_port, &GPIO_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;

		USART_Init(UART4, &USART_InitStructure);
		USART_Cmd(UART4, ENABLE);
	}
	else if (conf->_UARTx == UART5)
	{
		USART_DeInit(UART5);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

		RCC_AHB1PeriphClockCmd(conf->_tx->_rcc, ENABLE);
		GPIO_PinAFConfig(conf->_tx->_port, conf->_txSource, GPIO_AF_UART5);
		GPIO_InitStructure.GPIO_Pin = conf->_tx->_pin;
		GPIO_Init(conf->_tx->_port, &GPIO_InitStructure);

		RCC_AHB1PeriphClockCmd(conf->_rx->_rcc, ENABLE);
		GPIO_PinAFConfig(conf->_rx->_port, conf->_rxSource, GPIO_AF_UART5);
		GPIO_InitStructure.GPIO_Pin = conf->_rx->_pin;
		GPIO_Init(conf->_rx->_port, &GPIO_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;

		USART_Init(UART5, &USART_InitStructure);
		USART_Cmd(UART5, ENABLE);
	}

	if(!conf->_UseDMA || conf->_UARTx == UART4 || conf->_UARTx == UART5){
		USART_ITConfig(conf->_UARTx, USART_IT_RXNE, ENABLE);
		NVIC_Init(&NVIC_InitStructure);
	}
	setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    setPrintUART();
}

void UART::Print(const char* pstr, ...)
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

	if(Conf->_UARTx == USART1){
		if(!App::mApp->mUART1->isDmaBusy){
			App::mApp->mUART1->isDmaBusy = true;
			DMA_SetCurrDataCounter(DMA2_Stream7, length);
			DMA_Cmd(DMA2_Stream7, ENABLE);
		}
	}
	else if(Conf->_UARTx == USART3){
		if(!App::mApp->mUART3->isDmaBusy){
			App::mApp->mUART3->isDmaBusy = true;
			DMA_SetCurrDataCounter(DMA1_Stream3, length);
			DMA_Cmd(DMA1_Stream3, ENABLE);
		}
	}
	else if(Conf->_UARTx == UART4 || Conf->_UARTx == UART5){
		fp = txBuffer;
		for(int i = 0; i < length; i++){
			App::mApp->mTicks->setTimeout(3);
			while (USART_GetFlagStatus(Conf->_UARTx, USART_FLAG_TXE) == RESET){
				if(App::mApp->mTicks->Timeout()){
					return;
				}
			}
			USART_SendData(Conf->_UARTx, *(fp++));
		}
	}
}
