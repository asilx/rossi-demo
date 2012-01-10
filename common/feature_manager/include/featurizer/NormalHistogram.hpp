#ifndef NORMALHISTOGRAM_HPP_
#define NORMALHISTOGRAM_HPP_

#include "iostream"

namespace featurizer
{
	struct NormalHistogram
	{
		float histogram[20];
	};
	inline std::ostream& operator << (std::ostream& os, const NormalHistogram& p)
	{
		for (int i = 0; i < 20; ++i)
			os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 19 ? ", " : ")");
		return (os);
	}
}
POINT_CLOUD_REGISTER_POINT_STRUCT(
    featurizer::NormalHistogram,
    (float[20],histogram, histogram))

#endif
