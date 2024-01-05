// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "Random.h"
#define THISCLASS Random

RandomMersenneTwister *Random::smRandomMersenneTwister = 0;
RandomNormal *Random::smRandomNormal = 0;
RandomExponential *Random::smRandomExponential = 0;
RandomPoisson *Random::smRandomPoisson = 0;


THISCLASS::Random() {
	if (! smRandomMersenneTwister) {
		smRandomMersenneTwister = new RandomMersenneTwister();
		//std::cout << "MersenneTwister Random Number Generator initialized." << std::endl;
//		generator = new std::default_random_engine;
		Initialize();
	}
}

void THISCLASS::Initialize() {
	delete smRandomNormal;
	delete smRandomExponential;
	delete smRandomPoisson;
	smRandomNormal = new RandomNormal(smRandomMersenneTwister);
	smRandomExponential = new RandomExponential(smRandomMersenneTwister);
	smRandomPoisson = new RandomPoisson(smRandomMersenneTwister);
}

void THISCLASS::SaveState(std::ostream &out) {
	out << *smRandomMersenneTwister << std::endl;
	Initialize(); // Note that we need to do this in order to be at the same state after LoadState. (Disadvantage: SaveState modifies the current state.)
}

void THISCLASS::LoadState(std::istream &in) {
	in >> *smRandomMersenneTwister;
	Initialize();
}
