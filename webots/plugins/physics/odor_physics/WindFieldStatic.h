// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classWindFieldStatic
#define classWindFieldStatic

class WindFieldStatic;

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iostream>
#include "WindField.h"
#include "WindFieldSnapshot.h"

//!	WindFieldStatic
class WindFieldStatic: public WindField {

protected:
	//! Wind field snapshots.
	WindFieldSnapshot mWindFieldSnapshot;

public:
	//! Constructor.
	WindFieldStatic(Simulation *sim): WindField(sim) {}
	//! Destructor.
	~WindFieldStatic() {}

	//! Reads a static wind field.
	bool SetFile(const std::string &file);

	// WindField methods
	void OnSimulationStart() {}
	void OnSimulationEnd() {}
	void OnSimulationStep() {}
	void OnWebotsPhysicsDraw() {}
	Point3 GetWindSpeed(const Point3 &preal);
	void WriteConfiguration(std::ostream &out);
};

#endif
