// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classTextFileReaderDouble
#define classTextFileReaderDouble

class TextFileReaderDouble;

#include <string>
#include "TextFileReader.h"

//!	Reads double values from a text file.
class TextFileReaderDouble: public TextFileReader {

private:
	double *mpData;
	int mCols;
	int mRows;

public:
	// Constructor
	TextFileReaderDouble(const std::string &filename);

	// Read methods
	void Read(int width, int height);
	void ReadAll(int minwidth = 0, int minheight = 0);

	// Get methods
	double *GetData() {
		return mpData;
	}
	int GetCols() {
		return mCols;
	}
	int GetRows() {
		return mRows;
	}

	double Get(int col, int row) {
		//wxASSERT(col>=0);
		//wxASSERT(row>=0);
		//wxASSERT(col<mCols);
		//wxASSERT(row<mRows);
		return mpData[ColRowToI(col, row)];
	}

	double Get(int i) {
		//wxASSERT(i>=0);
		//wxASSERT(i<mCols*mRows);
		return mpData[i];
	}

	// Utility
	int ColRowToI(int col, int row) {
		return row*mCols + col;
	}
};

#endif
