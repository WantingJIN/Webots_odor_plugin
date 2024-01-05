// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classObstacleList
#define classObstacleList

class ObstacleList;

#include <string>
#include "Simulation.h"
#include "SimulationInterface.h"
#include "Point3.h"
#include "Obstacle.h"

//!	ObstacleList
class ObstacleList: public SimulationInterface {
	friend class TextFileReaderObstacleList;

protected:
	//! Array with obstacles.
	Obstacle *mObstacle;
	//! Allocated number of obstacles. This number is at least as big the number of requested obstacles.
	int mCountAllocated;
	//! Last added obstacle ID.
	int mLastAdded;

public:
	//! Constructor.
	ObstacleList(Simulation *sim, int count = 0);
	//! Destructor.
	~ObstacleList();

	//! Sets the number of filaments. Note that existing filaments will be deleted and a new set of filaments will be created. Note that the actual amount of filaments created may be bigger than the requested amount.
	void SetCount(int count);

	//! Reads wind speed information from a text file.
	void ReadTextFile(const std::string filename);

	//! Returns the wind speed at a specific point.
	Obstacle *AddObstacle();

	//! Returns the wind speed at a specific point.
	Obstacle *GetObstacle(const Point3 &preal);

	// SimulationInterface methods.
	void OnSimulationStart() {}
	void OnSimulationEnd() {}
	void OnSimulationStep() {}
	void OnWebotsPhysicsDraw() {}	// TODO: draw obstacles
	void WriteConfiguration(std::ostream &out);
};

#endif
