// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "TextFileWriter.h"
#define THISCLASS TextFileWriter

#include <time.h>

THISCLASS::TextFileWriter(const std::string& filename) {
	mBeginningOfLine = true;

	mError = 0;
	mFile.open(filename.c_str(), std::ios::out);
	if (! mFile.is_open()) {
		mError = cErrorFile;
		return;
	}
}

THISCLASS::~TextFileWriter() {
}

int THISCLASS::Error() {
	return mError;
}

void THISCLASS::Close() {
	mFile.close();
}

void THISCLASS::WriteComment(const std::string &s) {
	if (mBeginningOfLine) {
		mFile << "# " << s << std::endl;
	} else {
		mFile << "    # " << s << std::endl;
	}
	mBeginningOfLine = true;
}

void THISCLASS::WriteCommentField(const std::string &lb, const std::string &s) {
	if (! mBeginningOfLine) {
		mFile << std::endl;
		mBeginningOfLine = true;
	}
	mFile << "# " << lb << ":\t" << s << std::endl;
}

void THISCLASS::WriteCommentDate() {
	time_t t = time(0);
	char *tstr = ctime(&t);
	WriteCommentField("Date:", tstr);
}

void THISCLASS::Write(const std::string &s) {
	NextCol();
	mFile << s;
}
