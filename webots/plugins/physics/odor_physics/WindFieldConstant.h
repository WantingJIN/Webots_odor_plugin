// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classWindFieldConstant
#define classWindFieldConstant

class WindFieldConstant;

#include "WindField.h"

//!	WindFieldConstant
class WindFieldConstant: public WindField {

protected:
	// The constant wind speed vector.
	Point3 mWindSpeed;

public:
	//! Constructor.
	WindFieldConstant(Simulation *sim): WindField(sim), mWindSpeed(0, 0, 0) {}
	//! Destructor.
	~WindFieldConstant() {}

	//! Sets the constant wind speed.
	void SetWindSpeed(const Point3 &ws) {
		mWindSpeed = ws;
	}

	// WindField methods
	void OnSimulationStart() {}
	void OnSimulationEnd() {}
	void OnSimulationStep() {}
	void OnWebotsPhysicsDraw() {}
	Point3 GetWindSpeed(const Point3 &preal) {
		return mWindSpeed;
	}
	void WriteConfiguration(std::ostream &out);
};

#endif
