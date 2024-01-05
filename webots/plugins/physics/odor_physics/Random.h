// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classRandom
#define classRandom

class Random;

//#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include "RandomMersenneTwister.h"
#include "RandomNormal.h"
#include "RandomExponential.h"
#include "RandomPoisson.h"
#include <random>

//! A collection of random number functions.
class Random {

protected:
        
	//! The RandomMersenneTwister object.
	static RandomMersenneTwister* smRandomMersenneTwister;
	//! The RandomNormal object.
	static RandomNormal* smRandomNormal;
	//! The RandomExponential object.
	static RandomExponential* smRandomExponential;
	//! The RandomPoisson object.
	static RandomPoisson* smRandomPoisson;

	//! Initialization
	void Initialize();

public:
	//! Constructor.
	Random();
	//std::default_random_engine generator; //this line has been replaced by the two following ones
	std::random_device rd{};				//by Faezeh on March 19, 2020
	std::mt19937 generator{rd()};			//because generated random values were always the same


	//! Saves the current state of the RNG.
	void SaveState(std::ostream &in);
	//! Loads a previously saved RNG state.
	void LoadState(std::istream &in);

	//! Returns 0 or 1 with equal probability.
	int Binary() {
		return smRandomMersenneTwister->randInt() & 1;
		//return random() & 1;
	}

	//! Returns 1 with probability p and 0 with probability 1-p.
	int Binary(double p) {
		return (Uniform() < p) ? 1 : 0;
	}

	//! Returns an integer in the range [from, to) with uniform distribution.
	int Uniform(int from, int to) {
		if (from >= to) {
			return from;
		}
		return smRandomMersenneTwister->randInt(to - from - 1) + from;
		//return (int)((double)random() / ((double)RAND_MAX+1) * (double)(to-from)) + from;
	}

	//! Returns a double in the range [0, 1) with uniform distribution.
	double Uniform() {
		return smRandomMersenneTwister->randExc();
		//return (double)random() / ((double)RAND_MAX+1);
	}

	//! Returns a double in the range [from, to) with uniform distribution.
	double Uniform(double from, double to) {
		return smRandomMersenneTwister->randExc() * (to - from) + from;
		//return ((double)random() / ((double)RAND_MAX+1) * (double)(to-from)) + from;
	}

	//! Returns a float with a gaussian distribution.
	float Normal() {
		return smRandomNormal->Normal();
	}

	//! Returns a float with a gaussian distribution with a given standard deviation and mean.
	float Normal(float mean, float stddev) {
		//return smRandomNormal->Normal() * stddev + mean;
                  std::normal_distribution<float> distribution(mean,stddev);
                  float number = distribution(generator);
                  //printf("random number: %f\n",number);
                  return number;
	}

	//! Returns a float with an exponential distribution.
	float Exponential() {
		return smRandomExponential->Exponential();
	}

	//! Returns an integer with a Poisson distribution.
	long int Poisson(double mean) {
		return smRandomPoisson->Poisson(mean);
	}
};

#endif
