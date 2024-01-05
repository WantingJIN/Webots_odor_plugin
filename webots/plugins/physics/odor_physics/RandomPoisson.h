// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classRandomPoisson
#define classRandomPoisson

class RandomPoisson;

#include <cmath>
#include "RandomMersenneTwister.h"

//! Poisson random number generator. You should use this only through the Random class.
class RandomPoisson {

protected:
	RandomMersenneTwister *mRandomMersenneTwister;

public:
	//! Constructor.
	RandomPoisson(RandomMersenneTwister *rmt): mRandomMersenneTwister(rmt) {}

	//! Returns a integer with a poisson distribution of density L.
	long int Poisson(double L);
	long int PoissonInver(double L);
	long int PoissonRatioUniforms(double L);
	long int PoissonLow(double L);
	long int PoissonModeSearch(double L);
	long int PoissonPatchwork(double L);
	static double PoissonF(long int k, double l_nu, double c_pm);
	static double LnFac(long int n);
};

#endif
