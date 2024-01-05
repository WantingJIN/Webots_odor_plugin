// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classDataFileReader
#define classDataFileReader

class DataFileReader;

#include <string>
#include <iostream>
#include <fstream>

//!	Utility class to read binary files.
class DataFileReader {

public:
	// Member variables
	std::ifstream mFile;
	int mError;

	// Constants
	static const int ERROR_FILE = 1;
	static const int ERROR_FORMAT = 2;
	static const int FILEHEADER = 0x324D4953; // SIM2

	// Constructor
	DataFileReader(const std::string &filename);
	~DataFileReader();

	// General methods
	int Error();
	void Close();

	// Read methods
	bool Header();
	std::string ShortString();
	int *IntArray(int cc);
	void IntArray(int *pdata, int cc);
	float *FloatArray(int cc);
	void FloatArray(float *pdata, int cc);
	double *DoubleArray(int cc);
	void DoubleArray(double *pdata, int cc);

	int Read(char *buffer, off_t count) {
		mFile.read(buffer, count);
		return mFile.gcount();
	}

	bool Bool() {
		bool i = 0;
		mFile.read((char*)&i, sizeof(bool));
		return i;
	}

	unsigned char Char() {
		unsigned char i = 0;
		mFile.read((char*)&i, sizeof(unsigned char));
		return i;
	}

	int Int() {
		int i = 0;
		mFile.read((char*)&i, sizeof(int));
		return i;
	}

	float Float() {
		float i = 0;
		mFile.read((char*)&i, sizeof(float));
		return i;
	}

	double Double() {
		double i = 0;
		mFile.read((char*)&i, sizeof(double));
		return i;
	}

};

#endif
