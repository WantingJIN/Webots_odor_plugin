// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "DataFileReader.h"
#define THIS DataFileReader

THIS::DataFileReader(const std::string& filename) {
	mError = 0;
	mFile.open(filename.c_str(), std::ios::in);
	if (! mFile.is_open()) {
		mError = ERROR_FILE;
		return;
	}
	if (! this->Header()) {
		mError = ERROR_FORMAT;
		return;
	}
}

THIS::~DataFileReader() {
}

int THIS::Error() {
	return mError;
}

void THIS::Close() {
	mFile.close();
}

// Reads the header
bool THIS::Header() {
	int head = this->Int();
	if (head != FILEHEADER) {
		return false;
	}
	return true;
}

// Reads a string
std::string THIS::ShortString() {
	int len = (int)Char();
	if (len == 0) {
		return std::string("");
	}
	//wxPrintf("DataFileReader::String 1 len=%d\n", len);
	char buffer[len];
	//wxPrintf("DataFileReader::String 2\n");
	mFile.read(buffer, len);
	if (mFile.gcount() < len) {
		return "";
	}
	//wxPrintf("DataFileReader::String 3\n");
	return std::string(buffer, len);
}

// Reads an integer array.
int *THIS::IntArray(int cc) {
	if (cc < 1) {
		return NULL;
	}
	int *pdata = new int[cc];
	mFile.read((char*)pdata, sizeof(int)*cc);
	return pdata;
}

// Reads an integer array.
void THIS::IntArray(int *pdata, int cc) {
	if (cc < 1) {
		return;
	}
	mFile.read((char*)pdata, sizeof(int)*cc);
}

// Reads a float array.
float *THIS::FloatArray(int cc) {
	if (cc < 1) {
		return NULL;
	}
	float *pdata = new float[cc];
	mFile.read((char*)pdata, sizeof(float)*cc);
	return pdata;
}

// Reads a float array.
void THIS::FloatArray(float *pdata, int cc) {
	if (cc < 1) {
		return;
	}
	mFile.read((char*)pdata, sizeof(float)*cc);
}

// Reads a double array.
double *THIS::DoubleArray(int cc) {
	if (cc < 1) {
		return NULL;
	}
	double *pdata = new double[cc];
	mFile.read((char*)pdata, sizeof(double)*cc);
	return pdata;
}

// Reads a double array.
void THIS::DoubleArray(double *pdata, int cc) {
	if (cc < 1) {
		return;
	}
	mFile.read((char*)pdata, sizeof(double)*cc);
}
