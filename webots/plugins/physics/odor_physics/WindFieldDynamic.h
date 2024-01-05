// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classWindFieldDynamic
#define classWindFieldDynamic

class WindFieldDynamic;

#include <string>
#include "Point3.h"
#include "WindField.h"
#include "WindFieldSnapshot.h"

//!	WindFieldDynamic
class WindFieldDynamic: public WindField {

protected:
	//! The folder containing the samples.
	std::string mSamplesFolder;
	//! Wind field snapshots.
	WindFieldSnapshot *mWindFieldSnapshot[2];

	//! The size of the file heap.
	static const int mFileHeapMax = 3224;
	//! The file heap.
	double mFileHeap[mFileHeapMax];
	//! The number of files in the heap.
	int mFileHeapCount;

	//! Reads the next file.
	bool ReadNextFile();

	//! The interpolation factor for combining the vectors of snapshot 1 and 2
	double mTimeInterpolationFactor;

public:
	//! Constructor.
	WindFieldDynamic(Simulation *sim);
	//! Destructor.
	~WindFieldDynamic() {}

	//! Sets the folder containing the samples. The samples are supposed to be in a subfolder named after the corresponding simulation time.
	bool SetFolder(const std::string &folder) {
		mSamplesFolder = folder;
		return true;
	}

	// WindField methods.
	void OnSimulationStart();
	void OnSimulationEnd();
	void OnSimulationStep();
	void OnWebotsPhysicsDraw() {}
	Point3 GetWindSpeed(const Point3 &preal);
	void WriteConfiguration(std::ostream &out);
	
	void windSnapshotMemoryAllocation(WindFieldSnapshot *wfs);
};

#endif
