/*
 * Omega.h
 *
 *  Created on: 2014�~8��24��
 *      Author: YunKei
 */

#ifndef OMEGA_H_
#define OMEGA_H_

#include <Kalman.h>

namespace Sensors{

	class Omega{

		public:

			Omega();
			static Omega* getInstance();
			void Update();
			float getOmega(int);
			void setOmega(int, float);
			float getRawOmega(int index);
			bool getIsValided();

		private:
			bool isValided;
			float _Omega[3];
			float _RawOmega[3];
			Kalman* OmegaKalman[3];
	};
};

using namespace Sensors;


#endif /* OMEGA_H_ */
