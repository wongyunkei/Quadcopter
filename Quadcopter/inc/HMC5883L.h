/*
 * MPU6050.h
 *
 *  Created on: 2014¦~8¤ë23¤é
 *      Author: YunKei
 */

#ifndef  HMC5883L_H_
#define  HMC5883L_H_

#include <Kalman.h>

using namespace Math;

namespace Sensors{

	class HMC5883L{
		public:
			#define XSCALE	0.00264707822318913
			#define YSCALE	0.00292081657181901
			#define ZSCALE	0.0032969023676514
			#define XOFFSET -0.167149765712816
			#define YOFFSET -0.0170576026511972
			#define ZOFFSET 0.152827895166594

			enum ADDR{ADDRESS = 0x1e};
			enum REG{CFG_REG_A = 0x00,
					 CFG_REG_B = 0x01,
					 MODE_REG = 0x02,
					 DATA_OUT_X_MSB = 0x03,
					 DATA_OUT_X_LSB = 0x04,
					 DATA_OUT_Z_MSB = 0x05,
					 DATA_OUT_Z_LSB = 0x06,
					 DATA_OUT_Y_MSB = 0x07,
					 DATA_OUT_Y_LSB = 0x08,
					 STATUS_REG = 0x09,
					 ID_REGA = 0x0A,
					 ID_REGB = 0x0B,
					 ID_REGC = 0x0c};

			HMC5883L(float);
			static HMC5883L* getInstance();
			bool Update();
			void setRawMagneticField(int, float);
			float getRawMagneticField(int);
			void setRawAccOffset(int, float);
			float getRawAccOffset(int);
			void setRawAccScale(int, float);
			float getRawAccScale(int);
			void setRawOmega(int, float);
			float getRawOmega(int);
			void setRawOmegaOffset(int, float);
			float getRawOmegaOffset(int);
			bool getIsValided();
			float getRawHead();
			void setRawHead(float value);

		private:
			bool isValided;
			float Interval;
			float RawMagneticField[3];
			float RawMagneticFieldOffset[3];
			float RawMagneticFieldScale[3];
			float RawHead;
			float RawHeadOffset;
			int inited;
			bool CompassCal();
			void FastInitialization();
			Kalman* CompassKalman;
	};
};

#endif /*  MPU6050_H_ */
