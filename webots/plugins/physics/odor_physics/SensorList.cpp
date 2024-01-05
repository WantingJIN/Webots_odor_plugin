// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "SensorList.h"
#define	THISCLASS SensorList

THISCLASS::SensorList(Simulation *sim):
		SimulationInterface(sim), mSensors() {

	mSimulation->mSensorList = this;
}

THISCLASS::~SensorList() {
	if (mSimulation->mSensorList == this) {
		mSimulation->mSensorList = 0;
	}
}

void THISCLASS::OnSimulationStart() {
	tSensorList::iterator it = mSensors.begin();
	while (it != mSensors.end()) {
		Sensor *s = *it;
		s->OnSimulationStart();
		it++;
	}
}

void THISCLASS::OnSimulationEnd() {
	tSensorList::iterator it = mSensors.begin();
	while (it != mSensors.end()) {
		Sensor *s = *it;
		s->OnSimulationEnd();
		it++;
	}
}

void THISCLASS::OnSimulationStep() {
	tSensorList::iterator it = mSensors.begin();
	while (it != mSensors.end()) {
		Sensor *s = *it;
		//std::cout << 'o' << std::endl;
		s->OnSimulationStep();
		it++;
	}
}

void THISCLASS::OnWebotsPhysicsDraw() {
	tSensorList::iterator it = mSensors.begin();
	while (it != mSensors.end()) {
		Sensor *s = *it;
		s->OnWebotsPhysicsDraw();
		it++;
	}
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<SensorList>" << std::endl;
	tSensorList::iterator it = mSensors.begin();
	while (it != mSensors.end()) {
		Sensor *s = *it;
		s->WriteConfiguration(out);
		it++;
	}
	out << "</SensorList>" << std::endl;
}

void THISCLASS::AddSensor(Sensor *s) {
	// Add to list
	mSensors.push_back(s);
}
