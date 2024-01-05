// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classTextFileWriterDouble
#define classTextFileWriterDouble

class TextFileWriterDouble;

#include <string>
#include "TextFileWriter.h"

//!	Stores double values into a text file.
class TextFileWriterDouble: public TextFileWriter {

public:
	//! Constructor
	TextFileWriterDouble(const std::string &filename);

	// Write methods
	//! Adds one double value to the current line of the file and switches to the next column.
	void Write(double data);
	//! Adds a double array to the file. If rows is omitted, exactly one line is added.
	void Write(double *pdata, int cols, int rows = 1);
};

#endif
