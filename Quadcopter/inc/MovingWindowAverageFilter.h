/*
 * MovingWindowAverageFilter.h
 *
 *  Created on: 2015¦~3¤ë24¤é
 *      Author: YunKei
 */

#ifndef MOVINGWINDOWAVERAGEFILTER_H_
#define MOVINGWINDOWAVERAGEFILTER_H_

namespace Math{

	class MovingWindowAverageFilter{
		public:
			MovingWindowAverageFilter(int windowSize);
			void Update(double value);
			double getAverage();
			void Clear();
		private:
			double* buffer;
			int WindowSize;
			int bufferCount;
	};
};

using namespace Math;

#endif /* MOVINGWINDOWAVERAGEFILTER_H_ */
