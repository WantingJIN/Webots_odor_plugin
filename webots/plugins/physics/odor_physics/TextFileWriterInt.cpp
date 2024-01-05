// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "TextFileWriterInt.h"
#define THISCLASS TextFileWriterInt

THISCLASS::TextFileWriterInt(const std::string& filename):
		TextFileWriter(filename) {
}

// Appends one int value to the file
void THISCLASS::Write(int data) {
	NextCol();
	mFile << data;
}

// Writes an int array.
void THISCLASS::Write(int *pdata, int cols, int rows) {
	if (cols*rows < 1) {
		return;
	}

	int i = 0;
	for (int x = 0; x < cols; x++) {
		for (int y = 0; y < rows; y++) {
			NextCol();
			mFile << pdata[i];
			i++;
		}
		NextRow();
	}
}
