// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classOdorModel
#define classOdorModel

class OdorModel;

#include <string>
#include "Filament.h"
#include "Simulation.h"
#include "SimulationInterface.h"

//! OdorModel
//! \brief This class implements more or less the model presented in "Filament-based atmospheric dispersion model to achieve short time-scale structure of odor plumes" of Jay A. Farrell. However, instead of implementing our own advection model, we use a WindField class.
class OdorModel: public SimulationInterface {

public:
	//! The maximum radius in which filaments are considered.
	double mCutRadius;

	//! Constructor.
	OdorModel(Simulation *sim);
	//! Destructor.
	~OdorModel();

	// SimulationInterface methods.
	void OnSimulationStart() {}
	void OnSimulationEnd() {}
	void OnSimulationStep() {}
	void OnWebotsPhysicsDraw() {}
	void WriteConfiguration(std::ostream &out);

	//! Returns the current (physical) odor concentration at the specified point (without noise and distortion).
	double GetConcentration(const Point3 &point, int odortype);
	//! Samples the odor on grid points and writes them to file.
	void WriteConcentration(std::ostream &out, int odortype, Point3 pstart, Point3 pend, Point3 increment);
};

#endif
