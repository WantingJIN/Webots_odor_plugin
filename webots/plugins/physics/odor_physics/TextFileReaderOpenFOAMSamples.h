// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classTextFileReaderOpenFOAMSamples
#define classTextFileReaderOpenFOAMSamples

class TextFileReaderOpenFOAMSamples;

#include <string>
#include "TextFileReader.h"
#include "WindFieldSnapshot.h"

//!	Reads double values from a text file.
class TextFileReaderOpenFOAMSamples: public TextFileReader {

private:

public:
	// Constructor
	TextFileReaderOpenFOAMSamples(const std::string &filename);

	// Read methods
	void Read(WindFieldSnapshot *wfs);
	
};

#endif
