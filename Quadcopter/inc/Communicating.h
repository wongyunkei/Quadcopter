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
	            SWITCH_LIGHT
			};

			Communicating(USART_TypeDef*, bool);
			static Communicating* getInstant();
			//void CommunicatingPoll();
			void ReceivePoll();
			void SendPoll();
			void Execute(int, float);
			int getPrintType();
			void RFSend(int, float);
			float getCmdData();
			void setCmdData(float value);
			void SendCmd(unsigned char, float);
			void floatToBytes(float, unsigned char*);

		private:
			USART_TypeDef* Com;
			bool isRF;
			bool isToken;
			unsigned char Buffer[2048];
			unsigned char txBuffer[2048];
			int BufferCount;
			int txBufferCount;
			unsigned char Bytes[7];
			int PrintType;
			float CmdData;
			int Cmd;
			float Data;
	};
};

using namespace Communication;

#endif /* COMMUNICATING_H_ */
