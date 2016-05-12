/*
 * Communicating.cpp
 *
 *  Created on: 2014�~11��11��
 *      Author: YunKei
 */

#include <Communicating.h>

using namespace Communication;
using namespace Math;
using namespace Control;

Communicating::Com::Com(Interface interface, uint32_t addr, int index) : _interface(interface), Index(index){
	switch(interface){
		case __UART:
			_UART = (UART*)addr;
			break;
		case __SPI:
			_Spi = (Spi*)addr;
			break;
		case __I2C:
			break;
	}
}

Communicating::Communicating(Com* com) : count(0), _com(com), WatchDog(0), Cmd(0), Data(0),isToken(false), BufferCount(0), PrintType(0), CmdData(0), txBufferCount(0){
}

int Communicating::getTxBufferCount(){
	return txBufferCount;
}

void Communicating::ReceivePoll(){
	switch(_com->_interface){
		case Com::__UART:
			Length = _com->_UART->AvailableLength;
			_com->_UART->Read(Buffer + BufferCount, Length);
			break;
		case Com::__SPI:
			Length = _com->_Spi->AvailableLength;
			_com->_Spi->Read(Buffer + BufferCount, Length);
			break;
		case Com::__I2C:
			break;
	}

	BufferCount += Length;

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
	char D[txBufferCount];
	for(int i = 0; i < txBufferCount; i++){
		D[i] = txBuffer[i];
	}
	switch(_com->_interface){
		case Com::__UART:
			if((!_com->_UART->Conf->_UseDMA || !_com->_UART->isDmaBusy) && txBufferCount >= 4){
				if(_com->_UART->Conf->_UseDMA){
					_com->_UART->Print("%s\n", D);
				}
				else{
					_com->_UART->setPrintUART();
					printf("%s\n", D);
				}
			}
			break;
		case Com::__SPI:
			if(txBufferCount >= 4){
				_com->_Spi->Print(_com->Index, "%s\n", D);
			}
			break;
		case Com::__I2C:
			break;
	}
	txBufferCount = 0;
}

void Communicating::Execute(int cmd, float data){
	switch(cmd){

		case CMD::WATCHDOG:
			App::mApp->mControlling->clearWatchDogCount();
//			App::mApp->mCommunicating2->Send(0, data);
//			Acknowledgement();
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
			App::mApp->mControlling->XPosPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::ROLL_KI:
			App::mApp->mControlling->XPosPid->setKi(data);
			Acknowledgement();
			break;
		case CMD::ROLL_KD:
			App::mApp->mControlling->XPosPid->setKd(data);
			Acknowledgement();
			break;
		case CMD::PITCH_KP:
			App::mApp->mControlling->YPosPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::PITCH_KI:
			App::mApp->mControlling->YPosPid->setKi(data);
			Acknowledgement();
			break;
		case CMD::PITCH_KD:
			App::mApp->mControlling->YPosPid->setKd(data);
			Acknowledgement();
			break;
		case CMD::YAW_KP:
			App::mApp->mControlling->YawPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::YAW_KI:
			App::mApp->mControlling->YawPid->setKi(data);
			Acknowledgement();
			break;
		case CMD::YAW_KD:
			App::mApp->mControlling->YawPid->setKd(data);
			Acknowledgement();
			break;
		case CMD::RESET_ALL:
			App::mApp->mControlling->setStart(false);
			App::mApp->mControlling->setStarting(false);
			App::mApp->mControlling->setStopping(false);
			App::mApp->mControlling->StopAllMotors();
			if(App::mApp->mCompass != 0){
				App::mApp->mCompass->Reset();
			}
			App::mApp->mQuaternion->Reset();
			App::mApp->mEncoder1->Reset();
			App::mApp->mEncoder2->Reset();
			App::mApp->mEncoder3->Reset();
			App::mApp->mEncoder4->Reset();
			App::mApp->mEncoder5->Reset();
			App::mApp->mEncoder6->Reset();
			App::mApp->mLocalization->Reset();
			if(App::mApp->mEncoderYaw != 0){
				App::mApp->mEncoderYaw->Reset();
			}
			App::mApp->PathState = 0;
//			for(int i = 0; i < 500; i++){
//				App::mApp->mMPU6050->Update();
//				App::mApp->mHMC5883L->Update();
//				App::mApp->mAcceleration->Update();
//				App::mApp->mOmega->Update();
//				App::mApp->mCompass->Update();
//				App::mApp->mQuaternion->Update();
		//		Delay::DelayMS(2);
	//		}

//			App::mApp->mControlling->RollOffset = MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[0]);
//			App::mApp->mControlling->PitchOffset = MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[1]);
//			App::mApp->mControlling->YawOffset = MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[2]);
//			App::mApp->mControlling->Lift = 0;
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
			App::mApp->mControlling->Speed = data;
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
		case CMD::SET_X_TRANSLATION:
				App::mApp->mLocalization->setEncoderXTranslation(data);
				Acknowledgement();
				break;
		case CMD::SET_Y_TRANSLATION:
				App::mApp->mLocalization->setEncoderYTranslation(data);
				Acknowledgement();
				break;
		case CMD::MOTOR_KP:
					App::mApp->mControlling->Motor1->setKp(data);
					App::mApp->mControlling->Motor2->setKp(data);
					App::mApp->mControlling->Motor3->setKp(data);
					App::mApp->mControlling->Motor4->setKp(data);
					Acknowledgement();
					break;
		case CMD::MOTOR_KI:
					App::mApp->mControlling->Motor1->setKi(data);
					App::mApp->mControlling->Motor2->setKi(data);
					App::mApp->mControlling->Motor3->setKi(data);
					App::mApp->mControlling->Motor4->setKi(data);
					Acknowledgement();
					break;
		case CMD::FORWARD:
					App::mApp->mControlling->Forward();
					Acknowledgement();
					break;
		case CMD::BACKWARD:
					App::mApp->mControlling->Backward();
					Acknowledgement();
					break;
		case CMD::LEFT:
					App::mApp->mControlling->Left();
					Acknowledgement();
					break;
		case CMD::RIGHT:
					App::mApp->mControlling->Right();
					Acknowledgement();
					break;
		case CMD::PAUSE:
					App::mApp->mControlling->Pause();
					Acknowledgement();
					break;
		case CMD::CW:
					App::mApp->mControlling->CW();
					Acknowledgement();
					break;
		case CMD::CCW:
					App::mApp->mControlling->CCW();
					Acknowledgement();
					break;
		case CMD::MOVE:
					App::mApp->mControlling->Move(App::mApp->mControlling->Speed, data, 0);
					Acknowledgement();
					break;
		case CMD::MANUAL_MODE:
					App::mApp->mControlling->ManualMode = !App::mApp->mControlling->ManualMode;
					Acknowledgement();
					break;
		case CMD::RETURN_HOME:
					App::mApp->PathState = 999;
					Acknowledgement();
					break;
		case CMD::TEST:
					App::mApp->mCommunicating2->Send(9, 689);
					Acknowledgement();
					break;
		case CMD::CLAMPER_STOP_ALL:
					App::mApp->PeriodicCmd = 0;
					App::mApp->PeriodicData = data;
//					App::mApp->mCommunicating3->Send(0,0);
					Acknowledgement();
					break;
		case CMD::CLAMPER_RESET:
					App::mApp->PeriodicCmd = 1;
					App::mApp->PeriodicData = 0;
//					App::mApp->mCommunicating3->Send(1,0);
					Acknowledgement();
					break;
		case CMD::CLAMPER_START:
					App::mApp->PeriodicCmd = 2;
					App::mApp->PeriodicData = 0;
//					App::mApp->mCommunicating3->Send(2,0);
					Acknowledgement();
					break;
		case CMD::CLAMPER_SET_MOTOR1_TARGET:
					App::mApp->PeriodicCmd = 3;
					App::mApp->PeriodicData = data;
//					App::mApp->mCommunicating3->Send(3,data);
					Acknowledgement();
					break;
		case CMD::CLAMPER_SET_MOTOR2_TARGET:
					App::mApp->PeriodicCmd = 4;
					App::mApp->PeriodicData = data;
//					App::mApp->mCommunicating3->Send(4,data);
					Acknowledgement();
					break;
		case CMD::CLAMPER_SET_MOTOR3_TARGET:
					App::mApp->PeriodicCmd = 5;
					App::mApp->PeriodicData = data;
//					App::mApp->mCommunicating3->Send(5,data);
					Acknowledgement();
					break;
		case CMD::CLAMPER_WATCHDOG:
					App::mApp->PeriodicCmd = 6;
					App::mApp->PeriodicData = data;
//					App::mApp->mCommunicating3->Send(6,0);
					break;
		case CMD::SUCCESS:
					printf("SUCCESS:%g\r\n", data);
					//App::mApp->mCommunicating2->Send(CMD::SUCCESS, data);
					Acknowledgement();
					break;
		case CMD::CLAMPER_SET_HORIZONTAL:
					App::mApp->PeriodicCmd = 7;
					App::mApp->PeriodicData = data;
					Acknowledgement();
					break;
	}

}

void Communicating::Acknowledgement(){
	App::mApp->mLed1->Blink(true, 100, 2);
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
