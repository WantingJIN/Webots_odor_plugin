// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "TextFileReaderDouble.h"
#define THISCLASS TextFileReaderDouble

#include <sstream>
#include <string.h>

THISCLASS::TextFileReaderDouble(const std::string& filename):
		TextFileReader(filename) {

	mpData = NULL;
	mCols = 0;
	mRows = 0;
}

void THISCLASS::ReadAll(int mincols, int minrows) {
	// Get size
	int col;
	int row;
	Start();
	while (NextToken(&col, &row, NULL)) {
		if (mincols <= col) {
			mincols = col + 1;
		}
		if (minrows <= row) {
			minrows = row + 1;
		}
	}

	Read(mincols, minrows);
}

// Reads a double array.
void THISCLASS::Read(int cols, int rows) {
	// Initialize memory
	delete mpData;
	mpData = new double[cols*rows];
	memset(mpData, 0, cols*rows*sizeof(double));
	mCols = cols;
	mRows = rows;

	// Init variables
	int col;
	int row;
	std::string s;
	double val;

	// Read
	Start();
	while (NextToken(&col, &row, &s)) {
		if (col < cols && row < rows) {
			int i = ColRowToI(col, row);
			std::istringstream istr(s);
			istr >> val;
			mpData[i] = val;
		}
	}
}
