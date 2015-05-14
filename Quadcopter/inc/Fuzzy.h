/*
 * Fuzzy.h
 *
 *  Created on: 2015¦~4¤ë28¤é
 *      Author: YunKei
 */

#ifndef FUZZY_H_
#define FUZZY_H_

namespace Math{

	class Fuzzy{

		public:

			Fuzzy(int index, int setLength, double maxE, double maxEC, double maxU, double minE, double minEC, double minU, double maxKp, double maxKi, double maxKd, double** fuzzyTable);
			static Fuzzy* getInstance(int index);
			bool FuzzyAlgorithm(double, double, double*, double*, double*);

		private:
			double MaxE, MaxEC, MaxU;
			double MinE, MinEC, MinU;
			int SetLength;
			double MaxKp, MaxKi, MaxKd;
			double Ke, Kec, Ku;
			int E, EC, U;
			double** FuzzyTable;
			void Fuzzification(double, double);
			bool FuzzyDerivation();
			void Defuzzification(double*, double*, double*);

	};
};

using namespace Math;


#endif /* FUZZY_H_ */
