// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "WindField.h"
#define	THISCLASS WindField

THISCLASS::WindField(Simulation *sim): SimulationInterface(sim) {
	mSimulation->mWindField = this;
}

THISCLASS::~WindField() {
	
	if (mSimulation->mWindField == this) {
		mSimulation->mWindField = 0;
	}
}
