// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classTextFileReaderInt
#define classTextFileReaderInt

class TextFileReaderInt;

#include <string>
#include "TextFileReader.h"

//!	Reads int values from a text file.
class TextFileReaderInt: public TextFileReader {

private:
	int *mpData;
	int mCols;
	int mRows;

public:
	// Constructor
	TextFileReaderInt(const std::string &filename);

	// Read methods
	void Read(int width, int height);
	void ReadAll(int minwidth = 0, int minheight = 0);

	// Get methods
	int *GetData() {
		return mpData;
	}
	int GetCols() {
		return mCols;
	}
	int GetRows() {
		return mRows;
	}

	int Get(int col, int row) {
		//wxASSERT(col>=0);
		//wxASSERT(row>=0);
		//wxASSERT(col<mCols);
		//wxASSERT(row<mRows);
		return mpData[ColRowToI(col, row)];
	}

	int Get(int i) {
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
