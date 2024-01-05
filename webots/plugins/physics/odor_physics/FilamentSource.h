// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classFilamentSource
#define classFilamentSource

class FilamentSource;

#include <string>
#include "SimulationInterface.h"

//! Base class for filament sources.
class FilamentSource: public SimulationInterface {

public:
	//! File where source information (release, ...) is logged.
	std::ostream *mFile;

	//! Constructor.
	FilamentSource(Simulation *sim): SimulationInterface(sim), mFile(0) {}
	//! Destructor.
	virtual ~FilamentSource() {}

};

#endif
