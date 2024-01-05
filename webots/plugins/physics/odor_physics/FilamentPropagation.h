// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classFilamentPropagation
#define classFilamentPropagation

class FilamentPropagation;

#include <string>
#include "Filament.h"
#include "Simulation.h"
#include "SimulationInterface.h"

//! FilamentPropagation
//! \brief This class implements more or less the model presented in "Filament-based atmospheric dispersion model to achieve short time-scale structure of odor plumes" of Jay A. Farrell. However, instead of implementing our own advection model, we use a WindField class.
class FilamentPropagation: public SimulationInterface {

public:
	struct {
		double mStdDev;					//!< The standard deviation of the superposed stochastic process.
		double mFilamentGrowthGamma;	//!< The gamma parameter of the filament growth [m^2/s].
	} mConfiguration;

	//! Constructor.
	FilamentPropagation(Simulation *sim);
	//! Destructor.
	~FilamentPropagation();

	// SimulationInterface methods.
	void OnSimulationStart();
	void OnSimulationEnd();
	void OnSimulationStep();
	void OnWebotsPhysicsDraw() {}
	void WriteConfiguration(std::ostream &out);
};

#endif
