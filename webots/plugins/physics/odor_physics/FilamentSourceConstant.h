// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classFilamentSourceConstant
#define classFilamentSourceConstant

class FilamentSourceConstant;

#include "Point3.h"
#include "SimulationInterface.h"
#include "Simulation.h"
#include <ode/ode.h>

//! Creates a uniform rectangular filament distribution.
class FilamentSourceConstant: public FilamentSource {

public:
	//! Webots interface.
	struct {
		dGeomID mGeometryID;		//!< Geometry ID, which allows us to get the position.
	} mWebotsInterface;

	//! Sensor configuration.
	struct {
		double mRadius;				//!< Radius of the source.
		double mReleaseAmount;		//!< Number of filaments per second released by the source.
		double mFilamentAmount;		//!< Number of molecules in released filaments.
		double mFilamentWidth;		//!< Size of released filaments.
		int mFilamentOdorType;		//!< Odor type.
	} mConfiguration;

	//! Sensor state.
	struct {
		double mReleaseAmountAccumulator;		//!< Accumulates fractional release amounts.
	} mState;

	//! Constructor.
	FilamentSourceConstant(Simulation *sim);

	// Overwritten SimulationInterface methods
	void OnSimulationStart();
	void OnSimulationEnd();
	void OnSimulationStep();
	void OnWebotsPhysicsDraw();
	void WriteConfiguration(std::ostream &out);
};

#endif
