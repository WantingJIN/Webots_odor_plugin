// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classWindField
#define classWindField

class WindField;

#include "Point3.h"
#include "Simulation.h"
#include "SimulationInterface.h"

//!	WindField
class WindField: public SimulationInterface {

public:
	//! Constructor.
	WindField(Simulation *sim);
	//! Destructor.
	virtual ~WindField();

	//! Returns the wind speed at a specific point.
	virtual Point3 GetWindSpeed(const Point3 &preal) = 0;
};

#endif
