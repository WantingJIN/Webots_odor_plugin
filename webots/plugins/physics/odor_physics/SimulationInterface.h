// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classSimulationInterface
#define classSimulationInterface

#include <string>
#include <ostream>

class SimulationInterface;
class Simulation;

//! Simulation interface.
class SimulationInterface {

protected:
	//! The associated simulation.
	Simulation *mSimulation;

public:
	//! Constructor.
	SimulationInterface(Simulation *sim): mSimulation(sim) {}
	//! Destructor.
	virtual ~SimulationInterface() {}

	//! This method is invoked when the simulation starts.
	virtual void OnSimulationStart() = 0;
	//! This method is invoked to perform a simulation step.
	virtual void OnSimulationStep() = 0;
	//! This method is invoked when the simulation ends.
	virtual void OnSimulationEnd() = 0;
	//! This method is invoked to (re)draw the world.
	virtual void OnWebotsPhysicsDraw() = 0;
	//! Writes the properties of this object.
	virtual void WriteConfiguration(std::ostream &out) = 0;

	//! Adds an error to the list of errors (at the moment, this just prints the error message).
	void AddError(const std::string &msg);
};

#endif
