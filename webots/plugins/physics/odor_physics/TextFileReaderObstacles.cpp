// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "TextFileReaderObstacles.h"
#define THISCLASS TextFileReaderObstacles

#include <sstream>

THISCLASS::TextFileReaderObstacles(const std::string &filename):
		TextFileReader(filename) {

}

void THISCLASS::Read(ObstacleList *ol) {
	// Init variables
	int col;
	int row;
	std::string s;
	double val;

	// Go through the file and determine dimensions
	Cube cur;
	Start();
	while (NextToken(&col, &row, &s)) {
		std::istringstream istr(s);
		istr >> val;
		if (col == 0) {
			cur.a.x = val;
		} else if (col == 1) {
			cur.a.y = val;
		} else if (col == 2) {
			cur.a.z = val;
		} else if (col == 3) {
			cur.b.x = val;
		} else if (col == 4) {
			cur.b.y = val;
		} else if (col == 5) {
			cur.b.z = val;
			Obstacle *o = ol->AddObstacle();
			o->mCube = cur;
		}
	}
}
