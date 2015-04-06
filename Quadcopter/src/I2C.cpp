/*
 * I2C.cpp
 *
 *  Created on: 2014¦~8¤ë18¤é
 *      Author: YunKei
 */

#include <stdio.h>
#include <I2C.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_it.h>
#include <Ticks.h>
#include <Delay.h>
#include <Leds.h>
#include <AdditionalTools.h>

I2C* _mI2C1;
I2C* _mI2C2;

I2C::I2C(I2C_TypeDef* I2Cx, CLOCK clock){

	_I2Cx = I2Cx;
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	I2C_InitStruct.I2C_ClockSpeed = clock;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	if(I2Cx == I2C1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

		_mI2C1 = this;
	}
	else if(I2Cx == I2C2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

		_mI2C2 = this;
	}

	GPIO_Init(GPIOB, &GPIO_InitStruct);
	I2C_Init(I2Cx, &I2C_InitStruct);
	I2C_Cmd(I2Cx, ENABLE);
}

void I2C::ResetI2C(){
	Leds::getInstance()->Toggle(Leds::LED2);
	GPIO_InitTypeDef GPIO_InitStruct;
	if(_I2Cx == I2C1){
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	}
	else if(_I2Cx == I2C2){
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	}
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	if(_I2Cx == I2C1){
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	}
	else if(_I2Cx == I2C2){
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	}

	GPIO_Init(GPIOB, &GPIO_InitStruct);
	if(_I2Cx == I2C1){
		GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
		Delay::DelayUS(1);
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
		Delay::DelayUS(1);
	}
	else if(_I2Cx == I2C2){
		GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_RESET);
		Delay::DelayUS(1);
		GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_SET);
		Delay::DelayUS(1);
	}

	I2C_DeInit(_I2Cx);
	I2C(_I2Cx, I2C::SPEED_400K);
}

I2C* I2C::getInstance(I2C_TypeDef* I2Cx){
	I2C* pI2C;
	if(I2Cx == I2C1){
		pI2C = _mI2C1;
	}
	else if(I2Cx == I2C2){
		pI2C = _mI2C2;
	}
	return pI2C;
}

bool I2C::Write(uint8_t addr, uint8_t reg, uint8_t data){

	Ticks::getInstance()->setTimeout(3);
	while(I2C_GetFlagStatus(_I2Cx, I2C_FLAG_BUSY)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_GenerateSTART(_I2Cx, ENABLE);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_Send7bitAddress(_I2Cx, addr << 1, I2C_Direction_Transmitter);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_SendData(_I2Cx, reg);
	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_SendData(_I2Cx, data);
	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_GenerateSTOP(_I2Cx, ENABLE);
	return true;
}

uint8_t I2C::Read(uint8_t addr, uint8_t reg){

	Ticks::getInstance()->setTimeout(3);
	while(I2C_GetFlagStatus(_I2Cx, I2C_FLAG_BUSY)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return 0;
		}
	}
	I2C_GenerateSTART(_I2Cx, ENABLE);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return 0;
		}
	}
	I2C_Send7bitAddress(_I2Cx, addr << 1, I2C_Direction_Transmitter);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return 0;
		}
	}

	I2C_SendData(_I2Cx, reg);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return 0;
		}
	}

	I2C_GenerateSTART(_I2Cx, ENABLE);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return 0;
		}
	}
	I2C_Send7bitAddress(_I2Cx, addr << 1, I2C_Direction_Receiver);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return 0;
		}
	}

	I2C_AcknowledgeConfig(_I2Cx, DISABLE);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return 0;
		}
	}
	uint8_t value = I2C_ReceiveData(_I2Cx);
	I2C_GenerateSTOP(_I2Cx, ENABLE);

	return value;
}

bool I2C::BurstWrite(uint8_t addr, uint8_t reg, uint8_t length, uint8_t* pdata){

	Ticks::getInstance()->setTimeout(3);
	while(I2C_GetFlagStatus(_I2Cx, I2C_FLAG_BUSY)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
			}
	}
	I2C_GenerateSTART(_I2Cx, ENABLE);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_Send7bitAddress(_I2Cx, addr << 1, I2C_Direction_Transmitter);
	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_SendData(_I2Cx, reg);

	for(int i = 0; i < length; i++){
		Ticks::getInstance()->setTimeout(3);
		while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
			if(Ticks::getInstance()->Timeout()){
				ResetI2C();
				return false;
			}
		}
		I2C_SendData(_I2Cx, pdata[i]);
	}
	I2C_GenerateSTOP(_I2Cx, ENABLE);
	return true;
}

bool I2C::BurstRead(uint8_t addr, uint8_t reg, uint8_t length, uint8_t* pdata){

	Ticks::getInstance()->setTimeout(3);
	while(I2C_GetFlagStatus(_I2Cx, I2C_FLAG_BUSY)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_GenerateSTART(_I2Cx, ENABLE);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_Send7bitAddress(_I2Cx, addr << 1, I2C_Direction_Transmitter);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}

	I2C_SendData(_I2Cx, reg);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}

	I2C_GenerateSTART(_I2Cx, ENABLE);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_Send7bitAddress(_I2Cx, addr << 1, I2C_Direction_Receiver);

	Ticks::getInstance()->setTimeout(3);
	while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
		if(Ticks::getInstance()->Timeout()){
			ResetI2C();
			return false;
		}
	}

	for(int i = 0; i < length; i++){
		if(i == length - 1){
			I2C_AcknowledgeConfig(_I2Cx, DISABLE);
		}
		else if(i < length - 1){
			I2C_AcknowledgeConfig(_I2Cx, ENABLE);
		}
		Ticks::getInstance()->setTimeout(3);
		while(!I2C_CheckEvent(_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)){
			if(Ticks::getInstance()->Timeout()){
				ResetI2C();
				return false;
			}
		}
		pdata[i] = I2C_ReceiveData(_I2Cx);
	}
	I2C_GenerateSTOP(_I2Cx, ENABLE);

	return true;
}
