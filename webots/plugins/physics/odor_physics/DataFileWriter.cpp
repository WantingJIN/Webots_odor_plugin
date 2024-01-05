// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "DataFileWriter.h"
#define THIS DataFileWriter

THIS::DataFileWriter(const std::string& filename) {
	mError = 0;
	mFile.open(filename.c_str(), std::ios::out | std::ios::binary);
	if (! mFile.is_open()) {
		mError = ERROR_FILE;
		return;
	}
	this->Header();
}

THIS::~DataFileWriter() {
}

int THIS::Error() {
	return mError;
}

void THIS::Close() {
	mFile.close();
}

// Writes the header.
void THIS::Header() {
	this->Int(FILEHEADER);
}

// Writes a string.
void THIS::ShortString(const std::string &s) {
	int len = (int)s.length();
	if (len > 255) {
		len = 255;
	}
	//Char((unsigned char)len);
	mFile.write(s.c_str(), len);
}

// Writes an integer array.
void THIS::IntArray(int *pdata, int cc) {
	if (cc < 1) {
		return;
	}
	mFile.write((char*)pdata, sizeof(int)*cc);
}

// Writes a float array.
void THIS::FloatArray(float *pdata, int cc) {
	if (cc < 1) {
		return;
	}
	mFile.write((char*)pdata, sizeof(float)*cc);
}

// Writes a double array.
void THIS::DoubleArray(double *pdata, int cc) {
	if (cc < 1) {
		return;
	}
	mFile.write((char*)pdata, sizeof(double)*cc);
}
