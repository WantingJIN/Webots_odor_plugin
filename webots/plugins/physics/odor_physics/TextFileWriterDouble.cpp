// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "TextFileWriterDouble.h"
#define THISCLASS TextFileWriterDouble

THISCLASS::TextFileWriterDouble(const std::string& filename):
		TextFileWriter(filename) {
}

// Appends one double value to the file
void THISCLASS::Write(double data) {
	NextCol();
	mFile << data;
}

// Writes a double array.
void THISCLASS::Write(double *pdata, int cols, int rows) {
	if (cols*rows < 1) {
		return;
	}

	int i = 0;
	for (int y = 0; y < rows; y++) {
		for (int x = 0; x < cols; x++) {
			NextCol();
			mFile << pdata[i];
			i++;
		}
		NextRow();
	}
}
