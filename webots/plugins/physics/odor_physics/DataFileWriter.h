// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classDataFileWriter
#define classDataFileWriter

class DataFileWriter;

#include <string>
#include <iostream>
#include <fstream>

//! Utility class to write binary files.
class DataFileWriter {

public:
	// Member variables
	std::ofstream mFile;
	int mError;

	// Constants
	static const int ERROR_FILE = 1;
	static const int FILEHEADER = 0x324D4953; // SIM2

	// Constructor
	DataFileWriter(const std::string &filename);
	~DataFileWriter();

	// General methods
	int Error();
	void Close();

	// Write methods
	void Header();
	void ShortString(const std::string &s);
	void IntArray(int *pdata, int cc);
	void FloatArray(float *pdata, int cc);
	void DoubleArray(double *pdata, int cc);

	void Write(char* buffer, off_t count) {
		mFile.write(buffer, count);
	}

	void Bool(bool i) {
		mFile.write((char*)&i, sizeof(bool));
	}

	void Char(unsigned char i) {
		mFile.write((char*)&i, sizeof(unsigned char));
	}

	void Int(int i) {
		mFile.write((char*)&i, sizeof(int));
	}

	void Float(float i) {
		mFile.write((char*)&i, sizeof(float));
	}

	void Double(double i) {
		mFile.write((char*)&i, sizeof(double));
	}

};

#endif
