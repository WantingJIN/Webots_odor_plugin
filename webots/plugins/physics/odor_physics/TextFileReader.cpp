// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "TextFileReader.h"
#define THISCLASS TextFileReader

THISCLASS::TextFileReader(const std::string& filename) {
//std::cout << "TextFileReader constructor" << std::endl;
	mError = 0;
	mFile.open(filename.c_str(), std::ios::in);
	if (! mFile.is_open()) {
		mError = cErrorFile;
		return;
	}
	

	mState = sStateBeginningOfRow;
	mBufferLen = 1;
	mBufferPos = 1;
	mCol = 0;
	mRow = 0;
}

THISCLASS::~TextFileReader() {
}

int THISCLASS::Error() {
	return mError;
}

void THISCLASS::Close() {
	mFile.close();
}

void THISCLASS::Start() {
	mState = sStateBeginningOfRow;
	mBufferLen = 1;
	mBufferPos = 1;
	mCol = 0;
	mRow = 0;
}

bool THISCLASS::NextToken(int *col, int *row, std::string *s) {
//std::cout << "TextFileReader NextToken" << std::endl;
	char item[1024];
	int itemnw = 0;

	while (mBufferLen > 0) {
		while (mBufferPos < mBufferLen) {
			char ch = mBuffer[mBufferPos++];

			if (mState == sStateBeginningOfRow) {
				if (ch == '\n' || ch == '\r') {
				} else if (ch == '\t' || ch == ' ') {
				} else if (ch == '#') {
					mState = sStateComment;
				} else {
					item[0] = ch;
					itemnw = 1;
					mState = sStateDefault;
				}
			} else if (mState == sStateBeginningOfCol) {
				if (ch == '\n' || ch == '\r') {
					mState = sStateBeginningOfRow;
				} else if (ch == '\t' || ch == ' ') {
				} else if (ch == '#') {
					mState = sStateComment;
				} else {
					item[0] = ch;
					itemnw = 1;
					mState = sStateDefault;
				}
			} else if (mState == sStateComment) {
				if (ch == '\n' || ch == '\r') {
					mState = sStateBeginningOfRow;
				}
			} else {
				if (ch == '\n' || ch == '\r') {
					*col = mCol;
					*row = mRow;
					mCol = 0;
					mRow++;
					if (s) {
						*s = std::string(item, itemnw);
					}
					mState = sStateBeginningOfRow;
					return true;
				} else if (ch == '\t' || ch == ' ') {
					*col = mCol;
					*row = mRow;
					mCol++;
					if (s) {
						*s = std::string(item, itemnw);
					}
					mState = sStateBeginningOfCol;
					return true;
				} else if (ch == '#') {
					*col = mCol;
					*row = mRow;
					mCol = 0;
					mRow++;
					if (s) {
						*s = std::string(item, itemnw);
					}
					mState = sStateComment;
					return true;
				} else {
					if (itemnw < 1024) {
						item[itemnw] = ch;
						itemnw++;
					}
				}
			}
		}

		// Read new chunk
		mBufferPos = 0;
		mFile.read(mBuffer, sizeof(mBuffer));
		mBufferLen = mFile.gcount();
	}

	if (mState == sStateDefault) {
		*col = mCol;
		*row = mRow;
		if (s) {
			*s = std::string(item, itemnw);
		}
		mState = sStateBeginningOfRow;
		return true;
	}

	// at end of file
	return false;
}
