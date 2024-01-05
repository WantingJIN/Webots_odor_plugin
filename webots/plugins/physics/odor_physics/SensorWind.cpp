// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "SensorWind.h"
#include "FilamentList.h"
#include "Filament.h"
#include "Random.h"
#include "Constants.h"
#include <plugins/physics.h>
#define THISCLASS SensorWind

THISCLASS::SensorWind(Simulation *sim):
		Sensor(sim) {

	mWebotsInterface.mGeometryID = 0;
	mWebotsInterface.mChannel = 0;
	mConfiguration.mNoiseStdDev = 0;
	mConfiguration.mRunningAverageFactor = 0;
	mState.mWind = Point3(0, 0, 0);
}

THISCLASS::~SensorWind() {
}

void THISCLASS::OnSimulationStart() {
	mState.mWind = Point3(0, 0, 0);
}

void THISCLASS::OnSimulationEnd() {
}

void THISCLASS::OnSimulationStep() {
	// Get the position of the sensor
	const dReal *p = dGeomGetPosition(mWebotsInterface.mGeometryID);
	const Point3 position(p[0], p[1], p[2]);

	// Get the orientation of the sensor
	dQuaternion orientation;
	dGeomGetQuaternion(mWebotsInterface.mGeometryID, orientation);
	orientation[1] = -orientation[1];
	orientation[2] = -orientation[2];
	orientation[3] = -orientation[3];
	//std::cout << "Sensor quaternion: " << orientation[0] << " " << orientation[1] << " " << orientation[2] << " " << orientation[3] << std::endl;

	// Get the raw concentration at the current point
	Point3 wind = mSimulation->mWindField->GetWindSpeed(position);
	//std::cout << "Wind before: " << wind << std::endl;
	qRotateVector(orientation, wind);
	//std::cout << "Wind after: " << wind << std::endl;
	
	//printf("wind1 %f %f %f\n", wind.x, wind.y, wind.z);

	// Add noise
	if (mConfiguration.mNoiseStdDev > 0) {
		Random r;
		wind.x += r.Normal(0, mConfiguration.mNoiseStdDev);
		wind.y += r.Normal(0, mConfiguration.mNoiseStdDev);
		wind.z += r.Normal(0, mConfiguration.mNoiseStdDev);
	}
	//printf("wind2 %f %f %f\n", wind.x, wind.y, wind.z);
	// Measured concentration is a running average
	mState.mWind = mState.mWind * mConfiguration.mRunningAverageFactor + wind * (1 - mConfiguration.mRunningAverageFactor);
	//printf("wind3 %f %f %f\n", mState.mWind.x, mState.mWind.y, mState.mWind.z);
	// Send this value to the controller (double)
	dWebotsSend(mWebotsInterface.mChannel, &mState.mWind, sizeof(mState.mWind));
	mWebotsInterface.mLogFile << mState.mWind.x << "\t" << mState.mWind.y << "\t" << mState.mWind.z << std::endl;
}

void THISCLASS::OnWebotsPhysicsDraw() {/*
	// Get the position
	const dReal *p = dGeomGetPosition(mWebotsInterface.mGeometryID);
	const Point3 position(p[0], p[1], p[2]);

	// Get the orientation of the sensor
	dQuaternion orientation;
	dGeomGetQuaternion(mWebotsInterface.mGeometryID, orientation);

	// Rotate the wind vector
	Point3 wind = mState.mWind;
	qRotateVector(orientation, wind);

	// Draw the release area
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glLineWidth(1.0);
	glBegin(GL_LINES);
	glColor3f(0, 1, 0);
	glVertex3f(position.x, position.y, position.z);
	glVertex3f(position.x + wind.x, position.y + wind.y, position.z + wind.z);
	glEnd();
	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);*/
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<SensorWind>" << std::endl;
	out << "\t<NoiseStdDev>" << mConfiguration.mNoiseStdDev << "</NoiseStdDev>" << std::endl;
	out << "\t<RunningAverageFactor>" << mConfiguration.mRunningAverageFactor << "</RunningAverageFactor>" << std::endl;
	out << "</SensorWind>" << std::endl;
}
