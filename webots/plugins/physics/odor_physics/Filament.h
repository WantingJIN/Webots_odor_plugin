// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classFilament
#define classFilament

class Filament;
class FilamentList;

#include "Point3.h"

/*!
	Models a single Filament.
*/
class Filament {

public:
	//! Whether the filament exists or not.
	bool mExists;
	//! Position.
	Point3 mPosition;
	//! Previous Position.
	Point3 mPrevPosition;
	//! Amount of molecules.
	double mAmount;
	//! Width of the gaussian curve.
	double mWidth;
	//! Creation time (simulation time).
	double mCreationTime;
	//! Type of filament (chemical substance).
	int mOdorType;
	//! Unique ID of this filament.
	int mID;
	//! The filament list.
	FilamentList *mFilamentList;

	//! Constructor
	Filament();
	//! Destructor
	~Filament();
};

#endif
