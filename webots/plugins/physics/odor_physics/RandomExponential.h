// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classRandomExponential
#define classRandomExponential

class RandomExponential;

#include <cmath>
#include "RandomMersenneTwister.h"

//! Normal and Exponential random number generator based on the Ziggurat Method.
class RandomExponential {

protected:
	RandomMersenneTwister *mRandomMersenneTwister;
	unsigned long ke[256];
	float we[256], fe[256];

	float efix(unsigned long jz, unsigned long iz);

public:
	//! Constructor.
	RandomExponential(RandomMersenneTwister *rmt);

	//! Returns a double with an exponential distribution.
	float Exponential() {
		unsigned long jz = mRandomMersenneTwister->randInt();
		unsigned long iz = jz & 255;
		return (jz < ke[iz]) ? jz*we[iz] : efix(jz, iz);
	}
};

#endif
