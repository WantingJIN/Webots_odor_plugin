// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classSensorOdor
#define classSensorOdor

class SensorOdor;

#include "Sensor.h"
#include "Point3.h"
#include <fstream>
#include <ode/ode.h>

//! An odor sensor.
//! \brief This class implements the sensor model presented in "Filament-based atmospheric dispersion model to achieve short time-scale structure of odor plumes" of Jay A. Farrell.
class SensorOdor: public Sensor {

public:
	//! Webots interface.
	struct {
		dGeomID mGeometryID;			//!< Geometry ID, which allows us to get the position.
		int mChannel;					//!< Sender channel, which we will write the concentration value to.
		std::ofstream mLogFile;			//!< File to which we will write all sensor values.
	} mWebotsInterface;

	//! Sensor configuration.
	struct {
		int mOdorType;					//!< Odor type.
		double mNoiseStdDev;			//!< Standard deviation of the noise.
		double mRunningAverageFactor;	//!< Factor for running average (over time, i.e. at each invocation of SimulationStep this factor is used).
	} mConfiguration;

	//! Sensor state.
	struct {
		double mConcentration;			//!< The measured concentration.
	} mState;

	//! Constructor
	SensorOdor(Simulation *sim);
	//! Destructor
	~SensorOdor();

	// Sensor methods
	void OnSimulationStart();
	void OnSimulationEnd();
	void OnSimulationStep();
	void OnWebotsPhysicsDraw();
	void WriteConfiguration(std::ostream &out);
};

#endif
