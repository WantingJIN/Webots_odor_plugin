// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classSimulation
#define classSimulation

class Simulation;

#include "SimulationInterface.h"
#include "ObstacleList.h"
#include "WindField.h"
#include "FilamentList.h"
#include "FilamentPropagation.h"
#include "OdorModel.h"
#include "FilamentSourceList.h"
#include "SensorList.h"
#include <ode/ode.h>

//! Simulation.
class Simulation: public SimulationInterface {

public:
	//! Webots interface.
	struct {
		dWorldID mWorldID;
		dSpaceID mSpaceID;
	} mWebotsInterface;

	//! The time discretisation interval of the simulation.
	double mSimulationTimeStep;
	//! The current simulation time.
	double mSimulationTime;

	//! The path to the results
	std::string mResultsFolder;

	//! The list with obstacles.
	ObstacleList *mObstacleList;
	//! The wind field.
	WindField *mWindField;
	//! The filament list.
	FilamentList *mFilamentList;
	//! The filament propagation model.
	FilamentPropagation *mFilamentPropagation;
	//! The odor concentration model.
	OdorModel *mOdorModel;
	//! The list of sources.
	FilamentSourceList *mFilamentSourceList;
	//! The list of sensors.
	SensorList *mSensorList;

	//! Constructor.
	Simulation();
	//! Destructor.
	~Simulation() {}

	// SimulationInterface methods
	void OnSimulationStart();
	void OnSimulationStep();
	void OnSimulationEnd();
	void OnWebotsPhysicsDraw();
	void WriteConfiguration(std::ostream &out);
};

#endif
