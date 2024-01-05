// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classTextFileWriter
#define classTextFileWriter

class TextFileWriter;

#include <string>
#include <iostream>
#include <fstream>

//!	Stores values into a text file.
class TextFileWriter {

protected:
	// Member variables
	std::ofstream mFile;
	int mError;
	bool mBeginningOfLine;

	// Constants
	static const int cErrorFile = 1;

public:
	//! Constructor
	TextFileWriter(const std::string &filename);
	//! Destructor
	~TextFileWriter();

	// General methods
	//! Returns the last error.
	int Error();
	//! Closes the file.
	void Close();

	// Write comment methods
	//! Writes a comment.
	void WriteComment(const std::string &s);
	//! Writes a comment field (label, value).
	void WriteCommentField(const std::string &lb, const std::string &s);
	//! Writes the date as a comment field.
	void WriteCommentDate();

	// Other write methods
	//! Writes a string value to the current line of the file and switches to the next column.
	void Write(const std::string &s);
	//! Switches to the next column.
	void NextCol() {
		if (mBeginningOfLine) {
			mBeginningOfLine = false;
		} else {
			mFile << "\t";
		}
	}
	//! Switches to the next row.
	void NextRow() {
		mFile << std::endl;
		mBeginningOfLine = true;
	}
};

#endif
