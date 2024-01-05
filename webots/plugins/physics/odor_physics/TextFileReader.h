// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classTextFileReader
#define classTextFileReader

class TextFileReader;

#include <string>
#include <iostream>
#include <fstream>

//!	Reads values from a text file.
class TextFileReader {

public:
	// Member variables
	std::ifstream mFile;
	int mError;

	// States when reading a file
	enum eState {
		sStateDefault = 0,
		sStateComment,
		sStateBeginningOfRow,
		sStateBeginningOfCol,
	};

	// State variables
	eState mState;		// state
	int mCol;			// current column
	int mRow;			// current row
	char mBuffer[1024];	// current buffered chunk
	int mBufferLen;		// length of chunk
	int mBufferPos;		// next char in chunk to process

	// Constants
	static const int cErrorFile = 1;

	// Constructor
	TextFileReader(const std::string &filename);
	~TextFileReader();

	// General methods
	int Error();
	void Close();

	// Read methods
	void Start();
	bool NextToken(int *col, int *row, std::string *s = NULL);
};

#endif
