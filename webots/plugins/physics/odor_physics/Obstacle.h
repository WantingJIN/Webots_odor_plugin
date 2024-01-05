// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classObstacle
#define classObstacle

class Obstacle;

#include "Cube.h"

/*!
	Models a single obstacle
*/
class Obstacle {

protected:

public:
	//! Whether the filament exists or not.
	bool mExists;
	//! Type of filament (chemical substance).
	int mType;
	//! Position.
	Cube mCube;

	//! Constructor
	Obstacle();
	//! Destructor
	~Obstacle();

	//! Returns true if the point is in the obstacle and false otherwise.
	bool IsObstacle(const Point3 &preal) {
		return mCube.Inside(preal);
	}
};

#endif
