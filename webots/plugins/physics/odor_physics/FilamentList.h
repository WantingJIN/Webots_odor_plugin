// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classFilamentList
#define classFilamentList

class FilamentList;

#include <string>
#include "Filament.h"
#include "Simulation.h"
#include "SimulationInterface.h"

//!	FilamentList
class FilamentList: public SimulationInterface {

protected:
	//! Array with the filaments.
	Filament *mFilament;
	//! Allocated number of filaments. This number is at least as big the number of requested filaments.
	int mCountAllocated;

	//! The ID of the last added filament.
	int mLastAddedFilamentID;

	//! Allocates an array of filaments.
	void AllocateFilaments(int count);
	//! Initializes the filaments.
	void InitializeFilaments();

public:
	//! Constructor.
	FilamentList(Simulation *sim, int count = 0);
	//! Destructor.
	~FilamentList();

	//! Sets the number of filaments. Note that existing filaments will be deleted and a new set of filaments will be created. Note that the actual amount of filaments created may be bigger than the requested amount.
	void SetCount(int count);

	//! Adds a filament (activates an empty filament slot or reuses the oldest slot).
	Filament *AddFilament();
	//! Removes a filament (frees a filament slot).
	bool RemoveFilament(int id);

	//! Returns the number of filaments.
	int GetCount() const {
		return mCountAllocated;
	}
	//! Returns the filament array.
	Filament *GetFilaments() const {
		return mFilament;
	}
	//! Returns one filament.
	Filament *Get(int i) const {
		return &(mFilament[i]);
	}

	// Read/Write
	//! Reads the object from a file.
	void Read(DataFileReader &f);
	//! Writes the object to a file.
	void Write(DataFileWriter &f);

	//! Returns the concentration at p.
	double GetConcentration(const Point3 &p, int type = 0) const;

	// SimulationInterface methods.
	void OnSimulationStart();
	void OnSimulationEnd();
	void OnSimulationStep();
	void OnWebotsPhysicsDraw();
	void WriteConfiguration(std::ostream &out);

	// Report
	//! Writes the position of all filaments.
	void WritePosition(std::ostream &out);
};

#endif
