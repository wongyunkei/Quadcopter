/*
 * Communicating.cpp
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#include <Communicating.h>
#include <stm32f4xx_usart.h>
#include <stdio.h>
#include <PWM.h>
#include <Controlling.h>
#include <Pid.h>
#include <Quaternion.h>
#include <math.h>
#include <MathTools.h>
#include <Task.h>
#include <inttypes.h>
#include <SE3.h>
#include <Delay.h>
#include <Leds.h>
#include <NRF905.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <MPU6050.h>

Communicating* _com[4];

Communicating::Communicating(COM com, USART_TypeDef* usart, bool rf) : WatchDog(0), Cmd(0), Data(0), isRF(rf),isToken(false), BufferCount(0), PrintType(0), CmdData(0), txBufferCount(0){
	_com[com] = this;
	Com = usart;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_9);
}

void Communicating::clearWatchDog(){
	Controlling::getInstant()->clearWatchDogCount();
}

int Communicating::getWatchDog(){
	return WatchDog;
}

Communicating* Communicating::getInstant(int index){
	return _com[index];
}

int Communicating::getTxBufferCount(){
	return txBufferCount;
}

void Communicating::ReceivePoll(){
	int length;
	if(isRF){
		length = NRF905::getInstance()->getBufferCount();
//		NRF905::getInstance()->Read(Buffer + BufferCount, length);
	}else{
		length = Usart::getInstance(Com)->getBufferCount();
		Usart::getInstance(Com)->Read(Buffer + BufferCount, length);
	}

//	if(length == 0){
//		WatchDog++;
//	}
//	if(WatchDog > 1000){
//		WatchDog = 0;
//		GPIO_ResetBits(GPIOB, GPIO_Pin_9);
//		Delay::DelayMS(20);
//		GPIO_SetBits(GPIOB, GPIO_Pin_9);
//	}
	BufferCount += length;

	for(int j = 0; j < 10; j++){
		if(BufferCount > 0){
			for(int i = 0; i < BufferCount; i++){
				if(Buffer[i] == '$'){
					int tokenPos = i;
					if((BufferCount - tokenPos) >= 5){
						if(Buffer[i + 4] == '\n'){
							char ch[3] = {Buffer[i + 1], Buffer[i + 2], Buffer[i + 3]};
							for(int k = 0; k < BufferCount - tokenPos - 5; k++){
								Buffer[k] = Buffer[k + 5];
							}
							BufferCount -= 5;

							int d[3];
							d[0] = (int)ch[0] - 14;
							for(int l = 1; l < 3; l++){
								if(ch[l] < 0){
									d[l] = (int)ch[l] + 255;
								}
								else{
									d[l] = (int)ch[l] - 1;
								}
							}
							int halfInt = (((int)d[1]) << 8) | ((int)d[2]);
							Data = MathTools::HalfIntToFloat(halfInt);
							Cmd = d[0];
							Execute(Cmd, Data);
							break;
						}
						else{
							if((BufferCount - tokenPos) > 0){
								for(int k = tokenPos; k < BufferCount - tokenPos - 1; k++){
									Buffer[k] = Buffer[k + 1];
								}
								BufferCount -= tokenPos + 1;
							}
							else{
								BufferCount = 0;
							}
						}
					}
				}
			}
		}
		else{
			BufferCount = 0;
			break;
		}
	}
}


//
//
//	if(BufferCount > 0){
//		for(int i = 0; i < BufferCount; i++){
//			for(int j = 0; j < BufferCount; j++){
//				Usart::setPrintUsart(USART3);
//				printf("b%d:%d,", j, Buffer[j]);
//
//			}
//			if(Buffer[i] == '$'){
//				isToken = true;
//				if(BufferCount - i >= 3){
//
//					isToken = false;
//					for(int j = 0; j < 3; j++){
//						Bytes[j] = Buffer[i + j + 1] - 1;
//					}
//					Usart::setPrintUsart(USART3);
//					printf("b0:%d b1:%d b2:%d\n", Bytes[0], Bytes[1], Bytes[2]);
//					Cmd = Bytes[0];
//					int halfInt = ((int)Bytes[1] << 8) | (int)Bytes[2];
//					Data = MathTools::HalfIntToFloat(halfInt);
//					Usart::setPrintUsart(USART3);
//					printf("Cmd:%d Data:%g\n", Cmd, Data);
////					uint16_t checkSum = 0;
////					for(int j = 0; j < 5; j++){
////						checkSum += (Bytes[j]);
////					}
////					if(checkSum == (uint16_t)(((uint16_t)Bytes[5] << 8) | (uint16_t)Bytes[6])){
//
////						union{
////							float f;
////							uint32_t d;
////						} x;
////						x.d = ((uint32_t)((uint32_t)Bytes[4]) << 24) | (uint32_t)(((uint32_t)Bytes[3]) << 16) | (uint32_t)(((uint32_t)Bytes[2]) << 8) | (uint32_t)((uint32_t)Bytes[1]);
////						Data = x.f;
//						Execute(Cmd, Data);
////					}
//					BufferCount = 0;
//				}
//			}
//		}
//	}
//}

void Communicating::SendPoll(bool isUseDMA){
	if((!isUseDMA || !Usart::getInstance(Com)->getIsDmaBusy()) && txBufferCount >= 4){
		char D[txBufferCount];
		for(int i = 0; i < txBufferCount; i++){
			D[i] = txBuffer[i];
		}
		if(isRF){
			NRF905::getInstance()->Write("%s", D);
		}
		else{
			if(isUseDMA){
				Usart::getInstance(Com)->Print("%s\n", D);
			}
			else{
				Usart::setPrintUsart(Com);
				printf("%s\n", D);
			}
		}
		txBufferCount = 0;
//		for(int i = 0; i < txBufferCount; i++){
//			txBuffer[i] = txBuffer[i + 4];
//		}
	}
}

void Communicating::Execute(int cmd, float data){

	switch(cmd){

		case CMD::WATCHDOG:
			clearWatchDog();
			break;
		case CMD::PRINT_MODE:
			PrintType = (uint32_t)data;
			Send(4, (float)PrintType);
			break;
		case CMD::STOP:
			Controlling::getInstant()->Stopping();
			Send(4, 0);
			break;
		case CMD::POWER:
			for(int i = 0; i < 4; i++){
				PWM::getInstant()->Control(i, data);
			}
			Send(4, data);
			break;
		case CMD::START:
			Controlling::getInstant()->Starting();
			Send(4, 0);
			break;
		case CMD::MOTOR_KP:
			Pid::getInstance(8)->setPid(data, Pid::getInstance(8)->getPid(1), Pid::getInstance(8)->getPid(2));
			Pid::getInstance(9)->setPid(data, Pid::getInstance(9)->getPid(1), Pid::getInstance(9)->getPid(2));
			Pid::getInstance(10)->setPid(data, Pid::getInstance(10)->getPid(1), Pid::getInstance(10)->getPid(2));
			Pid::getInstance(11)->setPid(data, Pid::getInstance(11)->getPid(1), Pid::getInstance(11)->getPid(2));
			Send(4, data);
			break;
		case CMD::MOTOR_KD:
			Pid::getInstance(8)->setPid(Pid::getInstance(8)->getPid(0), Pid::getInstance(8)->getPid(1), data);
			Pid::getInstance(9)->setPid(Pid::getInstance(9)->getPid(0), Pid::getInstance(9)->getPid(1), data);
			Pid::getInstance(10)->setPid(Pid::getInstance(10)->getPid(0), Pid::getInstance(10)->getPid(1), data);
			Pid::getInstance(11)->setPid(Pid::getInstance(11)->getPid(0), Pid::getInstance(11)->getPid(1), data);
			Send(4, data);
			break;
		case CMD::ROLL_KP:
			Pid::getInstance(0)->setPid(data, Pid::getInstance(0)->getPid(1), Pid::getInstance(0)->getPid(2));
			Send(4, data);
			break;
		case CMD::ROLL_KI:
			Pid::getInstance(0)->setPid(Pid::getInstance(0)->getPid(0), data, Pid::getInstance(0)->getPid(2));
			Send(4, data);
			break;
		case CMD::ROLL_KD:
			Pid::getInstance(17)->setPid(data, 0, 0);
//			Pid::getInstance(0)->setPid(Pid::getInstance(0)->getPid(0), Pid::getInstance(0)->getPid(1), data);
			Send(4, data);
			break;
		case CMD::PITCH_KP:
			Pid::getInstance(1)->setPid(data, Pid::getInstance(1)->getPid(1), Pid::getInstance(1)->getPid(2));
			Send(4, data);
			break;
		case CMD::PITCH_KI:
			Pid::getInstance(1)->setPid(Pid::getInstance(1)->getPid(0), data, Pid::getInstance(1)->getPid(2));
			Send(4, data);
			break;
		case CMD::PITCH_KD:
			Pid::getInstance(18)->setPid(data, 0, 0);
//			Pid::getInstance(1)->setPid(Pid::getInstance(1)->getPid(0), Pid::getInstance(1)->getPid(1), data);
			Send(4, data);
			break;
		case CMD::RESET_ALL:
			Controlling::getInstant()->setStart(false);
			Controlling::getInstant()->setStarting(false);
			Controlling::getInstant()->setStopping(false);
			for(int i = 0; i < 4; i++){
				PWM::getInstant()->Control(i, 0);
			}
			Quaternion::getInstance(0)->resetQuaternion();
//			PX4FLOW::getInstance()->reset();
//			SE3::getInstance()->reset();
			Controlling::getInstant()->setTarget(0, 0);
			Controlling::getInstant()->setTarget(1, 0);
			Controlling::getInstant()->setTarget(2, 0);
			Controlling::getInstant()->setTarget(3, 0.3f);
			Controlling::getInstant()->setLift(0.0);
			Send(4, 0);
			break;

		case OFFSET0:
			Controlling::getInstant()->setOffset(0, Controlling::getInstant()->getOffset(0) + data);
			Send(4, data);
			break;
		case OFFSET1:
			Controlling::getInstant()->setOffset(1, Controlling::getInstant()->getOffset(1) + data);
			Send(4, data);
			break;
		case OFFSET2:
			Controlling::getInstant()->setOffset(2, Controlling::getInstant()->getOffset(2) + data);
			Send(4, data);
			break;
		case OFFSET3:
			Controlling::getInstant()->setOffset(3, Controlling::getInstant()->getOffset(3) + data);
			Send(4, data);
			break;
		case CMD::ROLL_OFFSET:
			Controlling::getInstant()->setRPYOffset(0, data);//Controlling::getInstant()->getRPYOffset(0) + data);
			Send(4, data);
			break;
		case CMD::PITCH_OFFSET:
			Controlling::getInstant()->setRPYOffset(1, data);//Controlling::getInstant()->getRPYOffset(1) + data);
			Send(4, data);
			break;
		case CMD::YAW_OFFSET:
			Controlling::getInstant()->setRPYOffset(2, data);//Controlling::getInstant()->getRPYOffset(2) + data);
			Send(4, data);
			break;
		case CMD::HIGH:
			for(int i = 0; i < 4; i++){
				switch(i){
					case 0:
						TIM_SetCompare1(TIM8, 2400);
						break;
					case 1:
						TIM_SetCompare2(TIM8, 2400);
						break;
					case 2:
						TIM_SetCompare3(TIM8, 2400);
						break;
					case 3:
						TIM_SetCompare4(TIM8, 2400);
						break;
				}
			}
			Send(4, 1);
			break;
		case CMD::LOW:

			for(int i = 0; i < 4; i++){
				switch(i){
					case 0:
						TIM_SetCompare1(TIM8, 700);
						break;
					case 1:
						TIM_SetCompare2(TIM8, 700);
						break;
					case 2:
						TIM_SetCompare3(TIM8, 700);
						break;
					case 3:
						TIM_SetCompare4(TIM8, 700);
						break;
				}
			}
			Send(4, 0);
			break;
		case CMD::INTIAL_POWER:
			for(int i = 0; i < 4; i++){
				Controlling::getInstant()->setInitRPM(data);
			}
			Send(4, data);
			break;
		case CMD::YAW_KP:
			Pid::getInstance(2)->setPid(data, Pid::getInstance(2)->getPid(1), Pid::getInstance(2)->getPid(2));
			Send(4, data);
			break;
		case CMD::YAW_KI:
			Pid::getInstance(2)->setPid(Pid::getInstance(2)->getPid(0), data, Pid::getInstance(2)->getPid(2));
			Send(4, data);
			break;
		case CMD::YAW_KD:

			Pid::getInstance(19)->setPid(data, 0, 0);
//			Pid::getInstance(5)->setPid(Pid::getInstance(5)->getPid(0) + data, Pid::getInstance(5)->getPid(1), Pid::getInstance(5)->getPid(2));
			Send(4, data);
			break;
//		case CMD::Q:
//			Quaternion::getInstance()->getKalman(0)->setQ(data);
//			Quaternion::getInstance()->getKalman(1)->setQ(data);
//			Quaternion::getInstance()->getKalman(2)->setQ(data);
//			Send(4, data);
//			break;
//		case CMD::R1:
//			Quaternion::getInstance()->getKalman(0)->setR1(data);
//			Quaternion::getInstance()->getKalman(1)->setR1(data);
//			Quaternion::getInstance()->getKalman(2)->setR1(data);
//			Send(4, data);
//			break;
//		case CMD::R2:
//			Quaternion::getInstance()->getKalman(0)->setR2(data);
//			Quaternion::getInstance()->getKalman(1)->setR2(data);
//			Quaternion::getInstance()->getKalman(2)->setR2(data);
//			Send(4, data);
//			break;
		case CMD::DRIFT_KP:
			Pid::getInstance(5)->setPid(data,Pid::getInstance(5)->getPid(1),Pid::getInstance(5)->getPid(2));
			Pid::getInstance(6)->setPid(data,Pid::getInstance(6)->getPid(1),Pid::getInstance(5)->getPid(2));
			Pid::getInstance(7)->setPid(data,Pid::getInstance(7)->getPid(1),Pid::getInstance(5)->getPid(2));
			Send(4, data);
			break;
		case CMD::DRIFT_KI:
			Pid::getInstance(5)->setPid(Pid::getInstance(5)->getPid(0),data,Pid::getInstance(5)->getPid(2));
			Pid::getInstance(6)->setPid(Pid::getInstance(6)->getPid(0),data,Pid::getInstance(5)->getPid(2));
			Pid::getInstance(7)->setPid(Pid::getInstance(7)->getPid(0),data,Pid::getInstance(5)->getPid(2));
			Send(4, data);
			break;
		case CMD::SWITCH_LIGHT:
			GPIO_ToggleBits(GPIOC, GPIO_Pin_0);
			break;
		case CMD::HIGHT_KP:
			Pid::getInstance(3)->setPid(data, Pid::getInstance(3)->getPid(1), Pid::getInstance(3)->getPid(2));
			Send(4, data);
			break;
		case CMD::HIGHT_KI:
			Pid::getInstance(3)->setPid(Pid::getInstance(3)->getPid(0), data, Pid::getInstance(3)->getPid(2));
			Send(4, data);
			break;
		case CMD::HIGHT_KD:
			Pid::getInstance(3)->setPid(Pid::getInstance(3)->getPid(0), Pid::getInstance(3)->getPid(1), data);
			Send(4, data);
			break;
		case CMD::X_KP:
			Pid::getInstance(12)->setPid(data, Pid::getInstance(12)->getPid(1), Pid::getInstance(12)->getPid(2));
			Send(4, data);
			break;
		case CMD::X_KI:
			Pid::getInstance(12)->setPid(Pid::getInstance(12)->getPid(0), data, Pid::getInstance(12)->getPid(2));
			Send(4, data);
			break;
		case CMD::X_KD:
			Pid::getInstance(12)->setPid(Pid::getInstance(12)->getPid(0), Pid::getInstance(12)->getPid(1), data);
			Send(4, data);
			break;
		case CMD::Y_KP:
			Pid::getInstance(13)->setPid(data, Pid::getInstance(13)->getPid(1), Pid::getInstance(13)->getPid(2));
			Send(4, data);
			break;
		case CMD::Y_KI:
			Pid::getInstance(13)->setPid(Pid::getInstance(13)->getPid(0), data, Pid::getInstance(13)->getPid(2));
			Send(4, data);
			break;
		case CMD::Y_KD:
			Pid::getInstance(13)->setPid(Pid::getInstance(13)->getPid(0), Pid::getInstance(13)->getPid(1), data);
			Send(4, data);
			break;
		case CMD::MAX_LIFT_VALUE:
			Controlling::getInstant()->setMaxLift(data);
			Send(4, data);
			break;
		case CMD::MIN_LIFT_VALUE:
			Controlling::getInstant()->setMinLift(data);
			Send(4, data);
			break;
		case CMD::LIFT:
			Controlling::getInstant()->setLift(data);
			Send(4, data);
			break;
		case CMD::TARGET_ROLL:
			Controlling::getInstant()->setTarget(0, data);
			Send(4, data);
			break;
		case CMD::TARGET_PITCH:
			Controlling::getInstant()->setTarget(1, data);
			Send(4, data);
			break;
		case CMD::TARGET_YAW:
			Controlling::getInstant()->setTarget(2, data);
			Send(4, data);
			break;
	}

}

uint32_t Communicating::getPrintType(){
	return PrintType;
}

float Communicating::getCmdData(){
	return CmdData;
}

void Communicating::setCmdData(float value){
	CmdData = value;
}

void Communicating::Send(int cmd, float data){
	char bytes[4];
	int halfInt = MathTools::FloatToHalfInt(data);
	bytes[0] = 0x24;
	bytes[1] = (char)(cmd + 14);
	bytes[2] = (char)(((halfInt & 0xff00) >> 8) + 1);
	bytes[3] = (char)((halfInt & 0x00ff) + 1);
	for(int i = 0; i < 4; i++){
		txBuffer[txBufferCount++] = bytes[i];
	}
}

void Communicating::floatToBytes(float f, char* bytes){
	union{
		float d;
		uint32_t u;
	} x;
	x.d = f;

	bytes[3] = (char)((x.u & 0xff000000) >> 24);
	bytes[2] = (char)((x.u & 0x00ff0000) >> 16);
	bytes[1] = (char)((x.u & 0x0000ff00) >> 8);
	bytes[0] = (char)((x.u & 0x000000ff) >> 0);
}
