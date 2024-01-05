// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classRandomNormal
#define classRandomNormal

class RandomNormal;

//#include <cmath>
#include <stdlib.h>
#include "RandomMersenneTwister.h"
#include <random>

//! Normal random number generator based on the Ziggurat Method.
class RandomNormal {

protected:
	RandomMersenneTwister *mRandomMersenneTwister;
	unsigned long kn[128];
	float wn[128], fn[128];

	float nfix(long hz, unsigned long iz);

public:
	//! Constructor.
	RandomNormal(RandomMersenneTwister *rmt);

	//! Returns a double with a gaussian distribution.
	float Normal() {
		long hz = mRandomMersenneTwister->randInt();
		unsigned long iz = hz & 127;
		return (abs(hz) < kn[iz]) ? hz*wn[iz] : nfix(hz, iz);
		/*std::default_random_engine generator;
                  std::normal_distribution<double> distribution(5.0,2.0);
                  float number = (float)distribution(generator);
                  return number;*/
	}
};

#endif
