/*
 * CC1101.h
 *
 *  Created on: 2014¦~8¤ë7¤é
 *      Author: YunKei
 */

#ifndef CC1101_H_
#define CC1101_H_

#include <Spi.h>
#include <AdditionalTools.h>

namespace Communication{

	class CC1101{

		public:

			enum REG{SRES = 0x30,
					 SFSTXON = 0x31,
					 SXOFF = 0x32,
					 SCAL = 0x33,
					 SRX = 0x34,
					 STX = 0x35,
					 SIDLE = 0x36,
					 SWOR =	0x38,
					 SPWD = 0x39,
					 SFRX = 0x3a,
					 SFTX = 0x3b,
					 SWORRST = 0x3c,
					 SNOP = 0x3d,

					 PATABLE = 0x3e,

					 SINGLE_BYTE_TX_FIFO = 0x3f,
					 MULTI_BYTE_TX_FIFO = 0x7f,
					 SINGLE_BYTE_RX_FIFO = 0xbf,
					 MULTI_BYTE_RX_FIFO = 0xff,

					 IOCFG2 = 0x00,
					 IOCFG1	= 0x01,
					 IOCFG0	= 0x02,


					 FIFOTHR = 0x03,

					 FIFOTHR_CLOSE_IN_RX_0dB = 0,
					 FIFOTHR_CLOSE_IN_RX_6dB = _BV(4),
					 FIFOTHR_CLOSE_IN_RX_12dB = _BV(5),
					 FIFOTHR_CLOSE_IN_RX_18dB = (_BV(5) | _BV(4)),
					 FIFOTHR_TX_61_RX_4 = 0,
					 FIFOTHR_TX_57_RX_8 = 1,
					 FIFOTHR_TX_53_RX_12 = 2,
					 FIFOTHR_TX_49_RX_16 = 3,
					 FIFOTHR_TX_45_RX_20 = 4,
					 FIFOTHR_TX_41_RX_24 = 5,
					 FIFOTHR_TX_37_RX_28 = 6,
					 FIFOTHR_TX_33_RX_32 = 7,
					 FIFOTHR_TX_29_RX_36 = 8,
					 FIFOTHR_TX_25_RX_40 = 9,
					 FIFOTHR_TX_21_RX_44 = 10,
					 FIFOTHR_TX_17_RX_48 = 11,
					 FIFOTHR_TX_13_RX_52 = 12,
					 FIFOTHR_TX_9_RX_56 = 13,
					 FIFOTHR_TX_5_RX_60 = 14,
					 FIFOTHR_TX_1_RX_64 = 15,


					 SYNC1 = 0x04,
					 SYNC0 = 0x05,
					 PKTLEN	= 0x06,


					 PKTCTRL1 = 0x07,

					 PKTCTRL1_CRC_AUTOFLUSH = _BV(3),
					 PKTCTRL1_APPEND_STATUS = _BV(2),
					 PKTCTRL1_ADR_CHK_NO_CHECK = 0,
					 PKTCTRL1_ADR_CHK_NO_BROADCAST = 1,
					 PKTCTRL1_ADR_CHK_00_BROADCAST = 2,
					 PKTCTRL1_ADR_CHK_00_AND_FF_BROADCAST = 3,


					 PKTCTRL0 = 0x08,

					 PKTCTRL0_WHITE_DATA = _BV(6),
					 PKTCTRL0_CRC_EN = _BV(2),
					 PKTCTRL0_PKT_FORMAT_NORMAL = 0,
					 PKTCTRL0_PKT_FORMAT_SYNC_SERIAL = _BV(4),
					 PKTCTRL0_PKT_FORMAT_RANDOM_TX = _BV(5),
					 PKTCTRL0_PKT_FORMAT_ASYNC_SERIAL = (_BV(5) | _BV(4)),
					 PKTCTRL0_LENGTH_CONFIG_FIX_LENGTH = 0,
					 PKTCTRL0_LENGTH_CONFIG_VAR_LENGTH = 1,
					 PKTCTRL0_LENGTH_CONFIG_INT_LENGTH = 2,


					 ADDR = 0x09,
					 CHANNR = 0x0a,
					 FSCTRL1 = 0x0b,
					 FSCTRL0 = 0x0c,
					 FREQ2 = 0x0d,
					 FREQ1 = 0x0e,
					 FREQ0 = 0x0f,
					 MDMCFG4 = 0x10,
					 MDMCFG3 = 0x11,


					 MDMCFG2 = 0x12,

					 MDMCFG2_MOD_FORMAT_2FSK = 0,
					 MDMCFG2_MOD_FORMAT_GFSK = _BV(4),
					 MDMCFG2_MOD_FORMAT_ASK_OOK = (_BV(5) | _BV(4)),
					 MDMCFG2_MOD_FORMAT_4FSK = _BV(6),
					 MDMCFG2_MOD_FORMAT_MSK	= (_BV(6) | _BV(5) | _BV(4)),
					 MDMCFG2_MANCHESTER_EN = _BV(3),
					 MDMCFG2_SYNC_MODE_NO_PREAM_SYNC = 0,
					 MDMCFG2_SYNC_MODE_15_16_SYNC_WORD = 1,
					 MDMCFG2_SYNC_MODE_16_16_SYNC_WORD = 2,
					 MDMCFG2_SYNC_MODE_30_32_SYNC_WORD = 	3,
					 MDMCFG2_SYNC_MODE_NO_PREA_SYNC_CA_SEN = 4,
					 MDMCFG2_SYNC_MODE_15_16_SYNC_CA_SEN = 5,
					 MDMCFG2_SYNC_MODE_16_16_SYNC_CA_SEN = 6,
					 MDMCFG2_SYNC_MODE_30_32_SYNC_CA_SEN = 7,


					 MDMCFG1 = 0x13,

					 MDMCFG1_FEC_EN = _BV(7),
					 MDMCFG1_NUM_PREAMBLE_2	= 0,
					 MDMCFG1_NUM_PREAMBLE_3	= _BV(4),
					 MDMCFG1_NUM_PREAMBLE_4	= _BV(5),
					 MDMCFG1_NUM_PREAMBLE_6	= (_BV(5) | _BV(4)),
					 MDMCFG1_NUM_PREAMBLE_8	= _BV(6),
					 MDMCFG1_NUM_PREAMBLE_12 = (_BV(6) | _BV(4)),
					 MDMCFG1_NUM_PREAMBLE_16 = (_BV(6) | _BV(5)),
					 MDMCFG1_NUM_PREAMBLE_24 = (_BV(6) | _BV(5) | _BV(4)),


					 MDMCFG0 = 0x14,
					 DEVIATN = 0x15,
					 MCSM2 = 0x16,


					 MCSM1 = 0x17,

					 MCSM1_CCA_MODE_ALWAYS = 0,
					 MCSM1_CCA_MODE_RSSI_BELOW_THRESHOLD = _BV(4),
					 MCSM1_CCA_MODE_RECEIVING_PACKET =_BV(5),
					 MCSM1_CCA_MODE_RSSI_BELOW_THRESHOLD_RECEIVING_PACKET =	(_BV(5) | _BV(4)),
					 MCSM1_RXOFF_MODE_IDLE = 0,
					 MCSM1_RXOFF_MODE_FSTXON = _BV(2),
					 MCSM1_RXOFF_MODE_TX = _BV(3),
					 MCSM1_RXOFF_MODE_STAY_RX = (_BV(3) | _BV(2)),
					 MCSM1_TXOFF_MODE_IDLE = 0,
					 MCSM1_TXOFF_MODE_FSTXON = 1,
					 MCSM1_TXOFF_MODE_STAY_TX = 2,
					 MCSM1_TXOFF_MODE_RX = 3,


					 MCSM0 = 0x18,

					 MCSM0_FS_AUTOCAL_NEVER	= 0,
					 MCSM0_FS_AUTOCAL_IDLE_TO_RX_TX = _BV(4),
					 MCSM0_FS_AUTOCAL_RX_TX_TO_IDLE_AUTO = _BV(5),
					 MCSM0_FS_AUTOCAL_4_RX_TX_TO_IDLE_AUTO = (_BV(5) | _BV(4)),
					 MCSM0_PO_TIMEOUT_EXPIRE_COUNT_1 = 0,
					 MCSM0_PO_TIMEOUT_EXPIRE_COUNT_16 = _BV(2),
					 MCSM0_PO_TIMEOUT_EXPIRE_COUNT_64 = _BV(3),
					 MCSM0_PO_TIMEOUT_EXPIRE_COUNT_256 = (_BV(3) | _BV(2)),



					 FOCCFG = 0x19,

					 FOCCFG_FOC_PRE_K = 0,
					 FOCCFG_FOC_PRE_2K = _BV(3),
					 FOCCFG_FOC_PRE_3K = _BV(4),
					 FOCCFG_FOC_PRE_4K = (_BV(4) | _BV(3)),
					 FOCCFG_FOC_LIMIT_0	= 0,
					 FOCCFG_FOC_LIMIT_BW_8 = 1,
					 FOCCFG_FOC_LIMIT_BW_4 = 2,
					 FOCCFG_FOC_LIMIT_BW_2 = 3,


					 BSCFG = 0x1a,

					 BSCFG_BS_PRE_KI_K = 0,
					 BSCFG_BS_PRE_KI_2K = _BV(6),
					 BSCFG_BS_PRE_KI_3K = _BV(7),
					 BSCFG_BS_PRE_KI_4K = (_BV(7) | _BV(6)),
					 BSCFG_BS_PRE_KP_K = 0,
					 BSCFG_BS_PRE_KP_2K	= _BV(4),
					 BSCFG_BS_PRE_KP_3K = _BV(5),
					 BSCFG_BS_PRE_KP_4K	= (_BV(5) | _BV(4)),
					 BSCFG_BS_LIMIT_0 = 0,
					 BSCFG_BS_LIMIT_3_125 = 1,
					 BSCFG_BS_LIMIT_6_25 = 2,
					 BSCFG_BS_LIMIT_12_5 = 3,

					 AGCTRL2 = 0x1b,
					 AGCTRL1 = 0x1c,
					 AGCTRL0 = 0x1d,
					 WOREVT1 = 0x1e,
					 WOREVT0 = 0x1f,
					 WORCTRL = 0x20,
					 FREND1 = 0x21,
					 FREND0 = 0x22,
					 FSCAL3 = 0x23,
					 FSCAL2 = 0x24,
					 FSCAL1 = 0x25,
					 FSCAL0 = 0x26,
					 RCCTRL1 = 0x27,
					 RCCTRL0 = 0x28,
					 FSTEST = 0x29,
					 PTEST = 0x2a,
					 AGCTEST = 0x2b,
					 TEST2 = 0x2c,
					 TEST1 = 0x2d,
					 TEST0 = 0x2e,

					 PARTNUM = 0xf0,
					 VERSION = 0xf1,
					 FREQEST = 0xf2,
					 LQI = 0xf3,
					 RSSI = 0xf4,
					 MARCSTATE = 0xf5,
					 WORTIME1 = 0xf6,
					 WORTIME0 = 0xf7,
					 PKTSTATUS = 0xf8,
					 VCO_VC_DAC	= 0xf9,
					 TXBYTES = 0xfa,
					 RXBYTES = 0xfb,
					 RCCTRL1_STATUS = 0xfc,
					 RCCTRL0_STATUS = 0xfd,



					 AGCCTRL2_MAX_DVGA_GAIN_ALL = 0,
					 AGCCTRL2_MAX_DVGA_GAIN_1 = _BV(6),
					 AGCCTRL2_MAX_DVGA_GAIN_2 = _BV(7),
					 AGCCTRL2_MAX_DVGA_GAIN_3 = (_BV(7) | _BV(6)),

					 AGCCTRL2_MAX_LNA_GAIN_ALL = 0,
					 AGCCTRL2_MAX_LNA_GAIN_2_6_dB = _BV(3),
					 AGCCTRL2_MAX_LNA_GAIN_6_1_dB = _BV(4),
					 AGCCTRL2_MAX_LNA_GAIN_7_4_dB = (_BV(4) | _BV(3)),
					 AGCCTRL2_MAX_LNA_GAIN_9_2_dB = _BV(5),
					 AGCCTRL2_MAX_LNA_GAIN_11_5_dB = (_BV(5) | _BV(3)),
					 AGCCTRL2_MAX_LNA_GAIN_14_6_dB = (_BV(5) | _BV(4)),
					 AGCCTRL2_MAX_LNA_GAIN_17_1_dB = (_BV(5) | _BV(4) | _BV(3)),

					 AGCCTRL2_MAGN_TARGET_24_dB = 0,
					 AGCCTRL2_MAGN_TARGET_27_dB = 1,
					 AGCCTRL2_MAGN_TARGET_30_dB = 2,
					 AGCCTRL2_MAGN_TARGET_33_dB = 3,
					 AGCCTRL2_MAGN_TARGET_36_dB = 4,
					 AGCCTRL2_MAGN_TARGET_38_dB	= 5,
					 AGCCTRL2_MAGN_TARGET_40_dB = 6,
					 AGCCTRL2_MAGN_TARGET_42_dB	= 7,


					 AGCCTRL1_CARRIER_SENSE_REL_THR_DISABLE = 0,
					 AGCCTRL1_CARRIER_SENSE_REL_THR_6_dB = _BV(4),
					 AGCCTRL1_CARRIER_SENSE_REL_THR_10_dB = _BV(5),
					 AGCCTRL1_CARRIER_SENSE_REL_THR_14_dB = (_BV(5) | _BV(4)),

					 AGCCTRL1_CARRIER_SENSE_ABS_THR_DISABLE = _BV(3),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_NEG_7_dB = (_BV(3) | _BV(0)),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_NEG_6_dB = (_BV(3) | _BV(1)),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_NEG_5_dB = (_BV(3) | _BV(1) | _BV(0)),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_NEG_4_dB = (_BV(3) | _BV(2)),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_NEG_3_dB = (_BV(3) | _BV(2) | _BV(0)),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_NEG_2_dB = (_BV(3) | _BV(2) | _BV(1)),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_NEG_1_dB = (_BV(3) | _BV(2) | _BV(1) | _BV(0)),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_0_dB = 0,
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_POS_1_dB = _BV(0),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_POS_2_dB = _BV(1),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_POS_3_dB = (_BV(1) | _BV(0)),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_POS_4_dB = _BV(2),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_POS_5_dB = (_BV(2) | _BV(0)),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_POS_6_dB = (_BV(2) | _BV(1)),
					 AGCCTRL1_CARRIER_SENSE_ABS_THR_POS_7_dB = (_BV(2) | _BV(1) | _BV(0))};

			enum STATUS{IDLE = 0,
						RX = 1,
						TX = 2,
						FSTXON = 3,
						CALIBRATE = 4,
						SETTLING = 5,
						RXFIFO_OVERFLOW = 6,
						TXFIFO_UNDERFLOW = 7};

			CC1101(uint8_t, uint8_t, uint16_t);
			static CC1101* getInstance();
			void Send(const char* format, ...);
			void setReceived();
			int Read(uint8_t*);
			void Receive();
			void setReceivedInterrupt(bool);
			uint8_t Strobe(REG);
			void SingleWrite(REG, uint8_t);
			uint8_t SingleRead(REG);
			void BurstWrite(REG, uint8_t, uint8_t*);
			void BurstRead(REG, uint8_t, uint8_t*);
			void setIsTxMode(bool);
			bool getIsTxMode();

		private:

			bool IsTxMode;

	};
};

using namespace Communication;

#endif /* CC1101_H_ */
