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
#include <NRF905.h>
#include <MathTools.h>
#include <Task.h>
#include <inttypes.h>

Communicating* _mCommunicating;

Communicating::Communicating(USART_TypeDef* com, bool rf) : Cmd(0), Data(0), isRF(rf),isToken(false), BufferCount(0), PrintType(0), CmdData(0), txBufferCount(0){
	_mCommunicating = this;
	Com = com;
}

Communicating* Communicating::getInstant(){
	return _mCommunicating;
}

void Communicating::ReceivePoll(){
	int length;
	if(isRF){
		length = NRF905::getInstance()->getBufferCount();
		NRF905::getInstance()->Read(Buffer + BufferCount, length);
	}else{
		length = Usart::getInstance(Com)->getBufferCount();
		Usart::getInstance(Com)->Read(Buffer + BufferCount, length);
	}
	BufferCount += length;
	if(BufferCount > 0){
		for(int i = 0; i < BufferCount; i++){
			if(Buffer[i] == '$'){
				isToken = true;
				if(BufferCount - i >= 7){

					isToken = false;
					for(int j = 0; j < 7; j++){
						Bytes[j] = Buffer[i + j + 1];
					}
					uint16_t checkSum = 0;
					for(int j = 0; j < 5; j++){
						checkSum += (Bytes[j] - 1);
					}
					if(checkSum == (uint16_t)(((uint16_t)(Bytes[5] - 1) << 8) | (uint16_t)(Bytes[6] - 1))){
						Cmd = Bytes[0] - 1;
						union{
							unsigned char b[4] = {};
							float d;
						} x;
						x.b[0] = Bytes[1] - 1;
						x.b[1] = Bytes[2] - 1;
						x.b[2] = Bytes[3] - 1;
						x.b[3] = Bytes[4] - 1;
						Data = x.d;
						Execute(Cmd, Data);
					}
					BufferCount = 0;
				}
			}
		}
	}
}

void Communicating::SendPoll(){
	if(txBufferCount >= 8){
		unsigned char D[9];
		for(int i = 0; i < 8; i++){
			D[i] = txBuffer[i];
		}
		D[9] = '\0';
		if(NRF905::getInstance()->Write("%s", D)){
			txBufferCount -= 8;
			for(int i = 0; i < txBufferCount; i++){
				txBuffer[i] = txBuffer[i + 8];
			}
		}
	}
}

void Communicating::Execute(int cmd, float data){
	switch(cmd){

		case CMD::STOP:
			Controlling::getInstant()->Stopping();
			if(isRF){
				Communicating::getInstant()->RFSend(4, 0);
			}
			else{
				Usart::getInstance(USART1)->Print("Stopped\n");
			}
			break;
		case CMD::POWER:

			for(int i = 0; i < 4; i++){
				PWM::getInstant()->Control(i, Controlling::getInstant()->RPM2PWM(i, data));
			}
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("RPM:%g\n", data);
			}
			Usart::getInstance(USART1)->Print("RPM:%d\n", (uint16_t)data);
			break;
		case CMD::START:
			Controlling::getInstant()->Starting();
			if(isRF){
			}
			else{
				printf("Started\n");
			}
			break;
		case CMD::MOTOR_KP:
			Pid::getInstance(8)->setPid(data, Pid::getInstance(8)->getPid(1), Pid::getInstance(8)->getPid(2));
			Pid::getInstance(9)->setPid(data, Pid::getInstance(9)->getPid(1), Pid::getInstance(9)->getPid(2));
			Pid::getInstance(10)->setPid(data, Pid::getInstance(10)->getPid(1), Pid::getInstance(10)->getPid(2));
			Pid::getInstance(11)->setPid(data, Pid::getInstance(11)->getPid(1), Pid::getInstance(11)->getPid(2));

			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("MotorKp:%g\n", data);
			}
			break;
		case CMD::MOTOR_KD:
			Pid::getInstance(8)->setPid(Pid::getInstance(8)->getPid(0), Pid::getInstance(8)->getPid(1), data);
			Pid::getInstance(9)->setPid(Pid::getInstance(9)->getPid(0), Pid::getInstance(9)->getPid(1), data);
			Pid::getInstance(10)->setPid(Pid::getInstance(10)->getPid(0), Pid::getInstance(10)->getPid(1), data);
			Pid::getInstance(11)->setPid(Pid::getInstance(11)->getPid(0), Pid::getInstance(11)->getPid(1), data);
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("MotorKd:%g\n", data);
			}
			break;
		case CMD::ROLL_KP:
			Pid::getInstance(0)->setPid(data, Pid::getInstance(0)->getPid(1), Pid::getInstance(0)->getPid(2));
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("KpRoll:%g\n", data);
			}
			break;
		case CMD::ROLL_KI:
			Pid::getInstance(0)->setPid(Pid::getInstance(0)->getPid(0), Pid::getInstance(0)->getPid(1) + data, Pid::getInstance(0)->getPid(2));
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("KiRoll:%g\n", data);
			}
			break;
		case CMD::ROLL_KD:
			Pid::getInstance(17)->setPid(data, 0, 0);
//			Pid::getInstance(0)->setPid(Pid::getInstance(0)->getPid(0), Pid::getInstance(0)->getPid(1), data);
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("KdRoll:%g\n", data);
			}
			break;
		case CMD::PITCH_KP:
			Pid::getInstance(1)->setPid(data, Pid::getInstance(1)->getPid(1), Pid::getInstance(1)->getPid(2));
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("KpPitch:%g\n", data);
			}
			break;
		case CMD::PITCH_KI:
			Pid::getInstance(1)->setPid(Pid::getInstance(1)->getPid(0), Pid::getInstance(1)->getPid(1) + data, Pid::getInstance(1)->getPid(2));
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("KiPitch:%g\n", data);
			}
			break;
		case CMD::PITCH_KD:
			Pid::getInstance(18)->setPid(data, 0, 0);
//			Pid::getInstance(1)->setPid(Pid::getInstance(1)->getPid(0), Pid::getInstance(1)->getPid(1), data);
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("KdPitch:%g\n");
			}
			break;
		case CMD::RESET_ALL:
			Controlling::getInstant()->setStart(false);
			Controlling::getInstant()->setStarting(false);
			Controlling::getInstant()->setStopping(false);
			for(int i = 0; i < 4; i++){
				PWM::getInstant()->Control(i, 0);
			}
			Quaternion::getInstance()->resetQuaternion();

			Controlling::getInstant()->setTarget(0, 0);
			Controlling::getInstant()->setTarget(1, 0);
			Controlling::getInstant()->setTarget(2, 0);
			if(isRF){
				Communicating::getInstant()->RFSend(4, 0);
			}
			else{
				Usart::getInstance(USART1)->Print("ResetAll");
			}
			break;

		case OFFSET0:
			Controlling::getInstant()->setOffset(0, Controlling::getInstant()->getOffset(0) + data);
			if(isRF){
				CmdData = Controlling::getInstant()->getOffset(0);
			}
			else{
				printf("offset0:%g\n", Controlling::getInstant()->getOffset(0));
			}
			break;
		case OFFSET1:
			Controlling::getInstant()->setOffset(1, Controlling::getInstant()->getOffset(1) + data);
			if(isRF){
				CmdData = Controlling::getInstant()->getOffset(1);
			}
			else{
				printf("offset1:%g\n", Controlling::getInstant()->getOffset(1));
			}
			break;
		case OFFSET2:
			Controlling::getInstant()->setOffset(2, Controlling::getInstant()->getOffset(2) + data);
			if(isRF){
				CmdData = Controlling::getInstant()->getOffset(2);
			}
			else{
			}
			break;
		case OFFSET3:
			Controlling::getInstant()->setOffset(3, Controlling::getInstant()->getOffset(3) + data);
			if(isRF){
				CmdData = Controlling::getInstant()->getOffset(3);
			}
			else{
				printf("offset3:%g\n", Controlling::getInstant()->getOffset(3));
			}
			break;
		case CMD::PRINT_MODE:
			if(PrintType++ == 8){
				PrintType = 0;
			}
			if(isRF){
				Communicating::getInstant()->RFSend(4, PrintType);
			}
			else{
				Usart::getInstance(USART1)->Print("PrintType:%d\n", PrintType);
			}
			break;
		case CMD::ROLL_OFFSET:
			Controlling::getInstant()->setRPYOffset(0, data);//Controlling::getInstant()->getRPYOffset(0) + data);
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				printf("RollOffset:%g\n", Controlling::getInstant()->getRPYOffset(0));
			}
			break;
		case CMD::PITCH_OFFSET:
			Controlling::getInstant()->setRPYOffset(1, data);//Controlling::getInstant()->getRPYOffset(1) + data);
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				printf("PitchOffset:%g\n", Controlling::getInstant()->getRPYOffset(1));
			}
			break;
		case CMD::YAW_OFFSET:
			Controlling::getInstant()->setRPYOffset(2, data);//Controlling::getInstant()->getRPYOffset(2) + data);
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				printf("YawOffset:%g\n", Controlling::getInstant()->getRPYOffset(2));
			}
			break;
		case CMD::WATCHDOG:
			Controlling::getInstant()->clearWatchDogCount();
			break;
		case CMD::HIGH:
			for(int i = 0; i < 4; i++){
				switch(i){
					case 0:
						TIM_SetCompare1(TIM8, PWM::getInstant()->getUpperLimit());
						break;
					case 1:
						TIM_SetCompare2(TIM8, PWM::getInstant()->getUpperLimit());
						break;
					case 2:
						TIM_SetCompare3(TIM8, PWM::getInstant()->getUpperLimit());
						break;
					case 3:
						TIM_SetCompare4(TIM8, PWM::getInstant()->getUpperLimit());
						break;
				}
			}
			if(isRF){
			}
			else{
			}
			printf("Max\n");
			break;
		case CMD::LOW:

			for(int i = 0; i < 4; i++){
				switch(i){
					case 0:
						TIM_SetCompare1(TIM8, PWM::getInstant()->getLowerLimit());
						break;
					case 1:
						TIM_SetCompare2(TIM8, PWM::getInstant()->getLowerLimit());
						break;
					case 2:
						TIM_SetCompare3(TIM8, PWM::getInstant()->getLowerLimit());
						break;
					case 3:
						TIM_SetCompare4(TIM8, PWM::getInstant()->getLowerLimit());
						break;
				}
			}
			if(isRF){
			}
			else{
			}
			printf("Min\n");
			break;
		case CMD::INTIAL_POWER:
			for(int i = 0; i < 4; i++){
				Controlling::getInstant()->setInitRPM(data);
			}
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("InitRPM:%g\n", (float)Controlling::getInstant()->getInitRPM());
			}
			break;
		case CMD::YAW_KP:
			Pid::getInstance(2)->setPid(Pid::getInstance(2)->getPid(0) + data, Pid::getInstance(2)->getPid(1), Pid::getInstance(2)->getPid(2));
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("KpYaw:%g\n", Pid::getInstance(2)->getPid(0));
			}
			break;
		case CMD::YAW_KI:
			Pid::getInstance(2)->setPid(Pid::getInstance(2)->getPid(0), Pid::getInstance(2)->getPid(1) + data, Pid::getInstance(2)->getPid(2));
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("KiYaw:%g\n", Pid::getInstance(2)->getPid(1));
			}
			break;
		case CMD::YAW_KD:

			Pid::getInstance(19)->setPid(data, 0, 0);
//			Pid::getInstance(2)->setPid(Pid::getInstance(2)->getPid(0) + data, Pid::getInstance(2)->getPid(1), Pid::getInstance(2)->getPid(2));
			if(isRF){
				Communicating::getInstant()->RFSend(4, data);
			}
			else{
				Usart::getInstance(USART1)->Print("KdYaw:%g\n", Pid::getInstance(2)->getPid(2));
			}
			break;
		case CMD::Q:
			Quaternion::getInstance()->getKalman(0)->setQ(data);
			Quaternion::getInstance()->getKalman(1)->setQ(data);
			Quaternion::getInstance()->getKalman(2)->setQ(data);
			Communicating::getInstant()->RFSend(4, data);
			break;
		case CMD::R1:
			Quaternion::getInstance()->getKalman(0)->setR1(data);
			Quaternion::getInstance()->getKalman(1)->setR1(data);
			Quaternion::getInstance()->getKalman(2)->setR1(data);
			Communicating::getInstant()->RFSend(4, data);
			break;
		case CMD::R2:
			Quaternion::getInstance()->getKalman(0)->setR2(data);
			Quaternion::getInstance()->getKalman(1)->setR2(data);
			Quaternion::getInstance()->getKalman(2)->setR2(data);
			Communicating::getInstant()->RFSend(4, data);
			break;
		case CMD::DRIFT_KP:
			Pid::getInstance(5)->setPid(data,Pid::getInstance(5)->getPid(1),Pid::getInstance(5)->getPid(2));
			Pid::getInstance(6)->setPid(data,Pid::getInstance(6)->getPid(1),Pid::getInstance(5)->getPid(2));
			Pid::getInstance(7)->setPid(data,Pid::getInstance(7)->getPid(1),Pid::getInstance(5)->getPid(2));
			Communicating::getInstant()->RFSend(4, data);
			break;
		case CMD::DRIFT_KI:
			Pid::getInstance(5)->setPid(Pid::getInstance(5)->getPid(0),data,Pid::getInstance(5)->getPid(2));
			Pid::getInstance(6)->setPid(Pid::getInstance(6)->getPid(0),data,Pid::getInstance(5)->getPid(2));
			Pid::getInstance(7)->setPid(Pid::getInstance(7)->getPid(0),data,Pid::getInstance(5)->getPid(2));
			Communicating::getInstant()->RFSend(4, data);
			break;
	}

}

int Communicating::getPrintType(){
	return PrintType;
}

float Communicating::getCmdData(){
	return CmdData;
}

void Communicating::setCmdData(float value){
	CmdData = value;
}

void Communicating::RFSend(int cmd, float data){
	unsigned char bytes[9];
	unsigned char b[4];
	floatToBytes(data, b);
	bytes[0] = 0x24;
	bytes[1] = (unsigned char)(cmd + 1);
	bytes[2] = (unsigned char)(b[0] + 1);
	bytes[3] = (unsigned char)(b[1] + 1);
	bytes[4] = (unsigned char)(b[2] + 1);
	bytes[5] = (unsigned char)(b[3] + 1);
	uint16_t checkSum = cmd;
	for (int i = 0; i < 4; i++)
	{
		checkSum += b[i];
	}
	bytes[6] = (unsigned char)(((checkSum & 0xff00) >> 8) + 1);
	bytes[7] = (unsigned char)((checkSum & 0x00ff) + 1);
	bytes[8] = (unsigned char)'\0';
	for(int i = 0; i < 8; i++){
		txBuffer[txBufferCount++] = bytes[i];
	}
}

//void Communicating::RFSend(char cmd, float *data){
//	union{
//		unsigned char b[4] = {};
//		float d;
//	} x;
//	x.d = data[0];
//	unsigned char D[21];
//	D[0] = '$';
//	D[1] = (unsigned char)(cmd + 1);
//	D[2] = (unsigned char)(x.b[0] + 1);
//	D[3] = (unsigned char)(x.b[1] + 1);
//	D[4] = (unsigned char)(x.b[2] + 1);
//	D[5] = (unsigned char)(x.b[3] + 1);
//
//	x.d = data[1];
//	D[6] = (unsigned char)(x.b[0] + 1);
//	D[7] = (unsigned char)(x.b[1] + 1);
//	D[8] = (unsigned char)(x.b[2] + 1);
//	D[9] = (unsigned char)(x.b[3] + 1);
//
//	x.d = data[2];
//	D[10] = (unsigned char)(x.b[0] + 1);
//	D[11] = (unsigned char)(x.b[1] + 1);
//	D[12] = (unsigned char)(x.b[2] + 1);
//	D[13] = (unsigned char)(x.b[3] + 1);
//
//	x.d = data[3];
//	D[14] = (unsigned char)(x.b[0] + 1);
//	D[15] = (unsigned char)(x.b[1] + 1);
//	D[16] = (unsigned char)(x.b[2] + 1);
//	D[17] = (unsigned char)(x.b[3] + 1);
//	uint16_t checkSum = 0;
//	for(int i = 1; i < 18; i++){
//		checkSum += (D[i] & 0x0f) + ((D[i] & 0xf0) >> 4);
//	}
//	D[18] = (unsigned char)((checkSum & 0x00ff) + 1);
//	D[19] = (unsigned char)(((checkSum & 0xff00) >> 8) + 1);
//	D[20] = '\n';
//
//	for(int i = 0; i < 21; i++){
//		txBuffer[txBufferCount++] = D[i];
//	}
//	//NRF905::getInstance()->Write("%s", D);
//}

void Communicating::floatToBytes(float f, unsigned char* bytes){
	union{
		float d;
		uint32_t u;
	} x;
	x.d = f;

	bytes[3] = (unsigned char)((x.u & 0xff000000) >> 24);
	bytes[2] = (unsigned char)((x.u & 0x00ff0000) >> 16);
	bytes[1] = (unsigned char)((x.u & 0x0000ff00) >> 8);
	bytes[0] = (unsigned char)((x.u & 0x000000ff) >> 0);
}

//void Communicating::SendCmd(unsigned char cmd, float data)
//{
//	unsigned char bytes[7];
//	unsigned char b[4];
//    floatToBytes(data, b);
//	bytes[0] = 0x24;
//	bytes[1] = (unsigned char)(cmd + 1);
//	bytes[2] = (unsigned char)(b[0] + 1);
//	bytes[3] = (unsigned char)(b[1] + 1);
//	bytes[4] = (unsigned char)(b[2] + 1);
//	bytes[5] = (unsigned char)(b[3] + 1);
//	bytes[6] = (unsigned char)'\0';
//	//Usart::getInstance(Com)->Print("%s\n", bytes);
//	Usart::setPrintUsart(USART1);
//	printf("%s\n", bytes);
//}
