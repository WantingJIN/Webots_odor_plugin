// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classSensor
#define classSensor

#include "SimulationInterface.h"
#include <string>

class Sensor;

//! Sensor interface.
class Sensor: public SimulationInterface {

public:
	//! Constructor.
	Sensor(Simulation *sim): SimulationInterface(sim) {}
	//! Destructor.
	virtual ~Sensor() {}

};

#endif
