// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "ObstacleList.h"
#include "TextFileReaderObstacles.h"
#define	THISCLASS ObstacleList

THISCLASS::ObstacleList(Simulation *sim, int count):
		SimulationInterface(sim), mObstacle(NULL), mCountAllocated(0), mLastAdded(-1) {

	SetCount(count);
	mSimulation->mObstacleList = this;
}

THISCLASS::~ObstacleList() {
	if (mSimulation->mObstacleList == this) {
		mSimulation->mObstacleList = 0;
	}
	SetCount(0);
}

void THISCLASS::SetCount(int count) {
	// If more obstacles than allocated are requested, create a new set of obstacles.
	if (count > mCountAllocated) {
		Obstacle *oldobstacle = mObstacle;
		mObstacle = new Obstacle[count];
		if (mCountAllocated > 0) {
			for (int i = 0; i < mCountAllocated; i++) {
				mObstacle[i] = oldobstacle[i];
			}
			delete [] oldobstacle;
		}
		mLastAdded = mCountAllocated - 1;
		mCountAllocated = count;
		return;
	}

	// If less than half of the allocated obstacles are requested, create a new set of obstacles.
	if (count*2 < mCountAllocated) {
		Obstacle *oldobstacle = mObstacle;
		if (count == 0) {
			mObstacle = 0;
		} else {
			mObstacle = new Obstacle[count];
			for (int i = 0; i < count; i++) {
				mObstacle[i] = oldobstacle[i];
			}
		}
		mCountAllocated = count;
		mLastAdded = 0;
		delete [] oldobstacle;
		return;
	}
}

Obstacle *THISCLASS::AddObstacle() {
	int id = mLastAdded + 1;
	if (id >= mCountAllocated) {
		id = 0;
	}
	while (id != mLastAdded) {
		if (! mObstacle[id].mExists) {
			mLastAdded = id;
			mObstacle[id].mExists = true;
			return &(mObstacle[id]);
		}
		if (id >= mCountAllocated) {
			id = 0;
		}
	}

	SetCount(mCountAllocated*2);
	return AddObstacle();
}

void THISCLASS::ReadTextFile(const std::string filename) {
	TextFileReaderObstacles f(filename);
	f.Read(this);
}

Obstacle *THISCLASS::GetObstacle(const Point3 &preal) {
	for (int i = 0; i < mCountAllocated; i++) {
		if (mObstacle[i].IsObstacle(preal)) {
			return &(mObstacle[i]);
		}
	}
	return 0;
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<ObstacleList>" << std::endl;
	out << "<Size>" << mCountAllocated << "</Size>" << std::endl;
	out << "</ObstacleList>" << std::endl;
}
