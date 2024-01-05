// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "FilamentSourceList.h"
#define	THISCLASS FilamentSourceList

THISCLASS::FilamentSourceList(Simulation *sim):
		SimulationInterface(sim), mFilamentSources() {

	mSimulation->mFilamentSourceList = this;
}

THISCLASS::~FilamentSourceList() {
	if (mSimulation->mFilamentSourceList == this) {
		mSimulation->mFilamentSourceList = 0;
	}
}

void THISCLASS::OnSimulationStart() {
	tFilamentSourceList::iterator it = mFilamentSources.begin();
	while (it != mFilamentSources.end()) {
		FilamentSource *fs = *it;
		fs->OnSimulationStart();
		it++;
	}
}

void THISCLASS::OnSimulationEnd() {
	tFilamentSourceList::iterator it = mFilamentSources.begin();
	while (it != mFilamentSources.end()) {
		FilamentSource *fs = *it;
		fs->OnSimulationEnd();
		it++;
	}
}

void THISCLASS::OnSimulationStep() {
	tFilamentSourceList::iterator it = mFilamentSources.begin();
	while (it != mFilamentSources.end()) {
		FilamentSource *fs = *it;
		//std::cout << 'o' << std::endl;
		fs->OnSimulationStep();
		it++;
	}
}

void THISCLASS::OnWebotsPhysicsDraw() {
	tFilamentSourceList::iterator it = mFilamentSources.begin();
	while (it != mFilamentSources.end()) {
		FilamentSource *fs = *it;
		fs->OnWebotsPhysicsDraw();
		it++;
	}
}

void THISCLASS::AddFilamentSource(FilamentSource *fs) {
	// Add to list
	mFilamentSources.push_back(fs);
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<FilamentSourceList>" << std::endl;
	tFilamentSourceList::iterator it = mFilamentSources.begin();
	while (it != mFilamentSources.end()) {
		FilamentSource *fs = *it;
		fs->WriteConfiguration(out);
		it++;
	}
	out << "</FilamentSourceList>" << std::endl;
}
