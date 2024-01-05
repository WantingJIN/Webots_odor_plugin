// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "SimulationInterface.h"
#define THISCLASS SimulationInterface

#include <iostream>

void THISCLASS::AddError(const std::string &msg) {
	std::cout << msg << std::endl;
}
