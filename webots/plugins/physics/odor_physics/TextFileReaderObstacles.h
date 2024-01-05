// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classTextFileReaderObstacles
#define classTextFileReaderObstacles

class TextFileReaderObstacles;

#include <string>
#include "TextFileReader.h"
#include "ObstacleList.h"

//!	Reads double values from a text file.
class TextFileReaderObstacles: public TextFileReader {

private:

public:
	// Constructor
	TextFileReaderObstacles(const std::string &filename);

	// Read methods
	void Read(ObstacleList *ol);
};

#endif
