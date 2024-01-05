// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classFilamentSourceList
#define classFilamentSourceList

class FilamentSourceList;

#include <list>
#include <string>
#include "Simulation.h"
#include "SimulationInterface.h"
#include "FilamentSource.h"

//!	FilamentSourceList
class FilamentSourceList: public SimulationInterface {
	friend class TextFileReaderFilamentSourceList;

protected:
	//! List type.
	typedef std::list<FilamentSource*> tFilamentSourceList;

	//! Array with robots.
	tFilamentSourceList mFilamentSources;

public:
	//! Constructor.
	FilamentSourceList(Simulation *sim);
	//! Destructor.
	~FilamentSourceList();

	//! Adds a filament source.
	void AddFilamentSource(FilamentSource *fs);

	// SimulationInterface methods.
	void OnSimulationStart();
	void OnSimulationEnd();
	void OnSimulationStep();
	void OnWebotsPhysicsDraw();
	void WriteConfiguration(std::ostream &out);
};

#endif
