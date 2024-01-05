// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classTextFileWriterInt
#define classTextFileWriterInt

class TextFileWriterInt;

#include <string>
#include "TextFileWriter.h"

//!	Stores int values into a text file.
class TextFileWriterInt: public TextFileWriter {

public:
	//! Constructor
	TextFileWriterInt(const std::string &filename);

	// Write methods
	//! Adds one int value to the current line of the file and switches to the next column.
	void Write(int data);
	//! Adds an int array to the file. If rows is omitted, exactly one line is added.
	void Write(int *pdata, int cols, int rows = 1);
};

#endif
