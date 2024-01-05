// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "Obstacle.h"
#include <cassert>
#define THISCLASS Obstacle

THISCLASS::Obstacle():
		mExists(false), mType(1), mCube() {

}

THISCLASS::~Obstacle() {
}
