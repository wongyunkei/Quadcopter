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

			Omega();
			static Omega* getInstance();
			void Update();
			double getOmega(int);
			void setOmega(int, double);
			double getRawOmega(int index);

		private:

			double _Omega[3];
			double _RawOmega[3];
			Kalman* OmegaKalman[3];
	};
};

using namespace Sensors;


#endif /* OMEGA_H_ */
