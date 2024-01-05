// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "Filament.h"
#include "FilamentList.h"
#define THISCLASS Filament

THISCLASS::Filament():
		mExists(false), mPosition(0, 0, 0), mAmount(1), mWidth(0), mID(0), mFilamentList(NULL) {

}

THISCLASS::~Filament() {
}
