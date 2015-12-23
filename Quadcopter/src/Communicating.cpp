/*
 * Communicating.cpp
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#include <Communicating.h>

using namespace Communication;
using namespace Math;
using namespace Control;

Communicating::Com::Com(Interface interface, uint32_t addr) : _interface(interface){
	switch(interface){
		case __UART:
			_UART = (UART*)addr;
			break;
		case __SPI:
			break;
		case __I2C:
			break;
	}
}

Communicating::Communicating(Com* com) : _com(com), WatchDog(0), Cmd(0), Data(0),isToken(false), BufferCount(0), PrintType(0), CmdData(0), txBufferCount(0){
}

int Communicating::getTxBufferCount(){
	return txBufferCount;
}

void Communicating::ReceivePoll(){
	int length;
	switch(_com->_interface){
		case Com::__UART:
			length = _com->_UART->getBufferCount();
			_com->_UART->Read(Buffer + BufferCount, length);
			printf("%s", Buffer + BufferCount);
			break;
		case Com::__SPI:
			break;
		case Com::__I2C:
			break;
	}

	BufferCount += length;

	for(int j = 0; j < 10; j++){
		if(BufferCount > 0){
			for(int i = 0; i < BufferCount; i++){
				if(Buffer[i] == '$'){
					int tokenPos = i;
					if((BufferCount - tokenPos) >= 4){
						char ch[3] = {Buffer[i + 1], Buffer[i + 2], Buffer[i + 3]};
						for(int k = 0; k < BufferCount - tokenPos - 4; k++){
							Buffer[k] = Buffer[k + 4];
						}
						BufferCount -= 4;

						int d[3];
						d[0] = (int)ch[0] - 48;
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
				}
			}
		}
		else{
			BufferCount = 0;
			break;
		}
	}
}

void Communicating::SendPoll(){
	switch(_com->_interface){
		case Com::__UART:
			if((!_com->_UART->Conf->_UseDMA || !_com->_UART->getIsDmaBusy()) && txBufferCount >= 4){
				char D[txBufferCount];
				for(int i = 0; i < txBufferCount; i++){
					D[i] = txBuffer[i];
				}
				if(_com->_UART->Conf->_UseDMA){
					_com->_UART->Print("%s\n", D);
				}
				else{
					_com->_UART->setPrintUART();
					printf("%s\n", D);
				}
				txBufferCount = 0;
			}
			break;
		case Com::__SPI:
			break;
		case Com::__I2C:
			break;
	}
}

void Communicating::Execute(int cmd, float data){

	switch(cmd){

		case CMD::WATCHDOG:
			App::mApp->mControlling->clearWatchDogCount();
			break;
		case CMD::PRINT_MODE:
			PrintType = data;
			Acknowledgement();
			break;
		case CMD::STOP:
			App::mApp->mControlling->Stopping();
			Acknowledgement();
			break;
		case CMD::POWER:
			App::mApp->mPWM->Control1(data);
			App::mApp->mPWM->Control2(data);
			App::mApp->mPWM->Control3(data);
			App::mApp->mPWM->Control4(data);
			Acknowledgement();
			break;
		case CMD::START:
			App::mApp->mControlling->Starting();
			Acknowledgement();
			break;
		case CMD::ROLL_KP:
			App::mApp->mControlling->RollPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::ROLL_KI:
			App::mApp->mControlling->RollPid->setKi(data*1000);
			Acknowledgement();
			break;
		case CMD::ROLL_KD:
			App::mApp->mControlling->KdRollPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::PITCH_KP:
			App::mApp->mControlling->PitchPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::PITCH_KI:
			App::mApp->mControlling->PitchPid->setKi(data*1000);
			Acknowledgement();
			break;
		case CMD::PITCH_KD:
			App::mApp->mControlling->KdPitchPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::YAW_KP:
			App::mApp->mControlling->YawPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::YAW_KI:
			App::mApp->mControlling->YawPid->setKi(data*1000);
			Acknowledgement();
			break;
		case CMD::YAW_KD:
			App::mApp->mControlling->KdYawPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::RESET_ALL:
			App::mApp->mControlling->setStart(false);
			App::mApp->mControlling->setStarting(false);
			App::mApp->mControlling->setStopping(false);
			App::mApp->mControlling->StopAllMotors();
			App::mApp->mCompass->Reset();
			App::mApp->mQuaternion->Reset();

			for(int i = 0; i < 500; i++){
				App::mApp->mMPU6050->Update();
				App::mApp->mHMC5883L->Update();
				App::mApp->mAcceleration->Update();
				App::mApp->mOmega->Update();
				App::mApp->mCompass->Update();
				App::mApp->mQuaternion->Update();
				Delay::DelayMS(2);
			}

			App::mApp->mControlling->RollOffset = MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[0]);
			App::mApp->mControlling->PitchOffset = MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[1]);
			App::mApp->mControlling->YawOffset = MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[2]);
			App::mApp->mControlling->Lift = 0;
			Acknowledgement();
			break;
		case CMD::ROLL_OFFSET:
			App::mApp->mControlling->setRollOffset(App::mApp->mControlling->getRollOffset() + data);
			Acknowledgement();
			break;
		case CMD::PITCH_OFFSET:
			App::mApp->mControlling->setPitchOffset(App::mApp->mControlling->getPitchOffset() + data);
			Acknowledgement();
			break;
		case CMD::YAW_OFFSET:
			App::mApp->mControlling->setYawOffset(App::mApp->mControlling->getYawOffset() + data);
			Acknowledgement();
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
			Acknowledgement();
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
			Acknowledgement();
			break;
		case CMD::MAX_LIFT_VALUE:
			App::mApp->mControlling->maxLift = data;
			Acknowledgement();
			break;
		case CMD::MIN_LIFT_VALUE:
			App::mApp->mControlling->minLift = data;
			Acknowledgement();
			break;
		case CMD::LIFT:
			App::mApp->mControlling->Lift = data;
			Acknowledgement();
			break;
		case CMD::TARGET_ROLL:
			App::mApp->mControlling->setRollTarget(data);
			Acknowledgement();
			break;
		case CMD::TARGET_PITCH:
			App::mApp->mControlling->setPitchTarget(data);
			Acknowledgement();
			break;
		case CMD::TARGET_YAW:
			App::mApp->mControlling->setYawTarget(data);
			Acknowledgement();
			break;
	}

}

void Communicating::Acknowledgement(){
	App::mApp->mLed1->Blink(true, 100, 2);
	App::mApp->mLed2->Blink(true, 100, 2);
	App::mApp->mLed3->Blink(true, 100, 2);
	App::mApp->mLed4->Blink(true, 100, 2);
}

void Communicating::Send(int cmd, float data){
	char bytes[4];
	int halfInt = MathTools::FloatToHalfInt(data);
	bytes[0] = 0x24;
	bytes[1] = (char)(cmd + 48);
	bytes[2] = (char)(((halfInt & 0xff00) >> 8) + 1);
	bytes[3] = (char)((halfInt & 0x00ff) + 1);
	for(int i = 0; i < 4; i++){
		txBuffer[txBufferCount++] = bytes[i];
	}
}
