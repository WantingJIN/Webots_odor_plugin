// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classWindFieldSnapshot
#define classWindFieldSnapshot

class WindFieldSnapshot;

#include <string>
#include "Point3.h"
#include "Point3Int.h"

//! WindFieldSnapshot
class WindFieldSnapshot {
	friend class WindFieldDynamic;
	friend class WindFieldStatic;
	friend class TextFileReaderOpenFOAMSamples;

protected:
	//! Time.
	double mTime;
	//! Wind speeds.
	Point3 *mWind;
	//! Dimensions of the wind speed array.
	Point3Int mArraySize;
	//! Origin of the wind speed grid.
	Point3 mOrigin;	
	//! End of the wind speed grid.
	Point3 mEnd;
	//! Distance between grid points.
	Point3 mGridSize;
	//  Index of nearest cell centre for every mGridSize, between mOrigin and mEnd (Fa)
	int *mIndexTable;	//size = mArraySize.x * mArraySize.y * mArraySize.z
	//  Center of all the cells containing one measurement (Fa)
	Point3 *mCellCentres;
	//  number of cells/measurements (Fa)
	int mCellNbr;

	//! Allocates the wind array.
	void AllocateArray(const Point3Int &arraysize);
	void AllocateArray(const int &arraysize);

	//! Translates XYZ coordinates to an array index. If the coordinate is outside the current wind field, -1 is returned.
	int XYZToI(const Point3 &p) const;

public:
	//! Constructor.
	WindFieldSnapshot();
	//! Destructor.
	~WindFieldSnapshot();

	//! Copy
	void WindFieldSnapshotCopy(WindFieldSnapshot &);

	//! Reads wind speed information from a text file.
	void ReadTextFile(const std::string filename);
	//! Reads wind speed information from a binary file.
	void ReadBinaryFile(const std::string filename);
	//! Writes wind speed information to a binary file.
	void WriteBinaryFile(const std::string filename);

	//! Returns the time.
	double GetTime() const {
		return mTime;
	}
	//! Returns array size.
	Point3Int GetArraySize() const {
		return mArraySize;
	}
	//! Returns the origin of the wind speed grid.
	Point3 GetOrigin() const {
		return mOrigin;
	}
	//! Returns the distance between grid points.
	Point3 GetGridSize() const {
		return mGridSize;
	}

	//! Sets the time.
	void SetTime(double set) {
		mTime = set;
	}
	//! Sets array size.
	void SetArraySize(const Point3Int &set) {
		mArraySize = set;
	}
	//! Sets the origin of the wind speed grid.
	void SetOrigin(const Point3 &set) {
		mOrigin = set;
	}
	//! Sets the distance between grid points.
	void SetGridSize(const Point3 &set) {
		mGridSize = set;
	}

	//! Returns the wind speed at a specific point.
	Point3 GetWindSpeed(const Point3 &preal) const;
	//! Returns the wind speed at one of the grid points.
	Point3 GetWindSpeed(const Point3Int &p) const;

	// Report
	//! Writes the properties of this object.
	void WriteConfiguration(std::ostream &out);
};

#endif
