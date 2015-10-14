/*
 * Omega.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef OMEGA_H_
#define OMEGA_H_

#include <Kalman.h>

namespace Sensors{

	class Omega{

		public:

			Omega(int index);
			static Omega* getInstance(int index);
			void Update();
			float getOmega(int channel);
			void setOmega(int channel, float value);
			float getRawOmega(int channel);
			bool getIsValided();

		private:
			bool isValided;
			int DevIndex;
			float _Omega[3];
			float _RawOmega[3];
			Kalman* OmegaKalman[3];
	};
};

using namespace Sensors;


#endif /* OMEGA_H_ */
