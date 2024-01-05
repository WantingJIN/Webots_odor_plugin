// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "SensorOdor.h"
#include "FilamentList.h"
#include "Filament.h"
#include "Random.h"
#include "Constants.h"
#include <plugins/physics.h>
#define THISCLASS SensorOdor

THISCLASS::SensorOdor(Simulation *sim):
		Sensor(sim) {

	mWebotsInterface.mGeometryID = 0;
	mWebotsInterface.mChannel = 0;
	mConfiguration.mOdorType = 0;
	mConfiguration.mNoiseStdDev = 0;
	mConfiguration.mRunningAverageFactor = 0;
	mState.mConcentration = 0;
}

THISCLASS::~SensorOdor() {
}

void THISCLASS::OnSimulationStart() {
	mState.mConcentration = 0;
}

void THISCLASS::OnSimulationEnd() {
}

void THISCLASS::OnSimulationStep() {
	// Get the position of the sensor
	const dReal *p = dGeomGetPosition(mWebotsInterface.mGeometryID);
	const Point3 position(p[0], p[1], p[2]);

	// Get the raw concentration at the current point
	double concentration = mSimulation->mOdorModel->GetConcentration(position, mConfiguration.mOdorType);

	// Add noise
	if (mConfiguration.mNoiseStdDev > 0) {
		Random r;
		concentration += r.Normal(0, mConfiguration.mNoiseStdDev);
	}

	// Measured concentration is a running average
	mState.mConcentration = mState.mConcentration * mConfiguration.mRunningAverageFactor + concentration * (1 - mConfiguration.mRunningAverageFactor);

	// Send this value to the controller (double) and log the same value
	dWebotsSend(mWebotsInterface.mChannel, &mState.mConcentration, sizeof(mState.mConcentration));
	mWebotsInterface.mLogFile << mState.mConcentration << std::endl;
}

void THISCLASS::OnWebotsPhysicsDraw() {/*
	// Get the position
	const dReal *p = dGeomGetPosition(mWebotsInterface.mGeometryID);
	const Point3 position(p[0], p[1], p[2]);

	// Draw the release area
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glLineWidth(1.0);
	glBegin(GL_POINTS);
	glColor3f(0, 1, 0);
	glVertex3f(position.x, position.y, position.z);
	glEnd();
	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);*/
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<SensorOdor>" << std::endl;
	out << "\t<OdorType>" << mConfiguration.mOdorType << "</OdorType>" << std::endl;
	out << "\t<NoiseStdDev>" << mConfiguration.mNoiseStdDev << "</NoiseStdDev>" << std::endl;
	out << "\t<RunningAverageFactor>" << mConfiguration.mRunningAverageFactor << "</RunningAverageFactor>" << std::endl;
	out << "</SensorOdor>" << std::endl;
}
