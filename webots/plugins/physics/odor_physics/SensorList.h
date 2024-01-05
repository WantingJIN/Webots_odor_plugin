// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classSensorList
#define classSensorList

class SensorList;

#include <list>
#include <string>
#include "Simulation.h"
#include "SimulationInterface.h"
#include "Sensor.h"

//!	SensorList
class SensorList: public SimulationInterface {

protected:
	//! List type.
	typedef std::list<Sensor*> tSensorList;

	//! Array with robots.
	tSensorList mSensors;

public:
	//! Constructor.
	SensorList(Simulation *sim);
	//! Destructor.
	~SensorList();

	//! Adds a sensor.
	void AddSensor(Sensor *s);

	// SimulationInterface methods.
	void OnSimulationStart();
	void OnSimulationEnd();
	void OnSimulationStep();
	void OnWebotsPhysicsDraw();
	void WriteConfiguration(std::ostream &out);
};

#endif
