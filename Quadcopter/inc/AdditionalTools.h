/*
 * AdditionalTools.h
 *
 *  Created on: 2014¦~8¤ë7¤é
 *      Author: YunKei
 */

#ifndef ADDITIONALTOOLS_H_
#define ADDITIONALTOOLS_H_

#include <inttypes.h>

#define _BV(X)					(1L << X)
#define MSB(X)					((X & 0xff00) >> 8)
#define LSB(X)					(X & 0x00ff)

namespace Utility{
	class AdditionalTools{
		public:
			static void setBuffer(int index, float* buf, int length);
			static float* getBuffer(int index);
			static void printfBuffer(int index, int length);
		private:
	};
};

using namespace Utility;

#endif /* ADDITIONALTOOLS_H_ */
