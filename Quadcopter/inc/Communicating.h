/*
 * Communicating.h
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#ifndef COMMUNICATING_H_
#define COMMUNICATING_H_

#include <Usart.h>
#include <stm32f4xx_usart.h>
#include <inttypes.h>

namespace Communication{

	class Communicating{

		public:

			enum COM{COM1,COM2,COM3,COM4,COM5,COM6};

			enum CMD{
				WATCHDOG,
				PRINT_MODE,
				HIGH,
				LOW,
				RESET_ALL,
				START,
				STOP,
				POWER,
				INTIAL_POWER,
				ROLL_OFFSET,
				PITCH_OFFSET,
				YAW_OFFSET,
				ROLL_KP,
				ROLL_KI,
				ROLL_KD,
				PITCH_KP,
				PITCH_KI,
				PITCH_KD,
				YAW_KP,
				YAW_KI,
				YAW_KD,
				OFFSET0,
				OFFSET1,
				OFFSET2,
				OFFSET3,
				MOTOR_KP,
				MOTOR_KD,
				Q,
				R1,
				R2,
				DRIFT_KP,
				DRIFT_KI,
				SWITCH_LIGHT,
				HIGHT_KP,
				HIGHT_KI,
				HIGHT_KD,
				X_KP,
				X_KI,
				X_KD,
				Y_KP,
				Y_KI,
				Y_KD,
				MAX_LIFT_VALUE,
				MIN_LIFT_VALUE,
				LIFT,
				TARGET_ROLL,
				TARGET_PITCH,
				TARGET_YAW
			};

			Communicating(COM com, USART_TypeDef*, bool);
			static Communicating* getInstant(int index);
			void ReceivePoll();
			void SendPoll(bool isUseDMA = true);
			void Execute(int, float);
			uint32_t getPrintType();
			void Send(int, float);
			float getCmdData();
			void setCmdData(float value);
			void SendCmd(char, float);
			void floatToBytes(float, char*);
			int getTxBufferCount();
			void clearWatchDog();
			int getWatchDog();

		private:
			USART_TypeDef* Com;
			bool isRF;
			int WatchDog;
			bool isToken;
			char Buffer[2048];
			char txBuffer[2048];
			int txBufferCount;
			int BufferCount;
			char Bytes[7];
			uint32_t PrintType;
			float CmdData;
			int Cmd;
			float Data;
	};
};

using namespace Communication;

#endif /* COMMUNICATING_H_ */
