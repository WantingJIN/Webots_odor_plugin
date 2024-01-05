// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "FilamentSourceConstant.h"
#include "FilamentList.h"
#include "Random.h"
#include "Constants.h"
#include <cassert>
#include <plugins/physics.h>
#define	THISCLASS FilamentSourceConstant

THISCLASS::FilamentSourceConstant(Simulation *sim):
		FilamentSource(sim) {

	mWebotsInterface.mGeometryID = 0;
	mConfiguration.mRadius = 0;
	mConfiguration.mReleaseAmount = 1;
	mConfiguration.mFilamentAmount = 1;
	mConfiguration.mFilamentWidth = 1;
	mConfiguration.mFilamentOdorType = 0;
	mState.mReleaseAmountAccumulator = 0;
}

void THISCLASS::OnSimulationStart() {}

void THISCLASS::OnSimulationEnd() {}

void THISCLASS::OnSimulationStep() {
	// Get the position of the source
	const dReal *p = dGeomGetPosition(mWebotsInterface.mGeometryID);
	const Point3 position(p[0], p[1], p[2]);

	// Accumulate fractional filaments
	mState.mReleaseAmountAccumulator += mConfiguration.mReleaseAmount * mSimulation->mSimulationTimeStep;
	if (mState.mReleaseAmountAccumulator < 1) {
		return;
	}

	// Create filaments
	Random r;
	double radius2 = mConfiguration.mRadius * mConfiguration.mRadius;
	FilamentList *fl = mSimulation->mFilamentList;
	while (mState.mReleaseAmountAccumulator >= 1) {
		Filament *f = fl->AddFilament();
		f->mCreationTime = mSimulation->mSimulationTime;
		f->mWidth = mConfiguration.mFilamentWidth;
		f->mOdorType = mConfiguration.mFilamentOdorType;
		f->mAmount = mConfiguration.mFilamentAmount;
		while (1) {
			f->mPosition.x = r.Uniform(-mConfiguration.mRadius, mConfiguration.mRadius);
			f->mPosition.y = r.Uniform(-mConfiguration.mRadius, mConfiguration.mRadius);
			f->mPosition.z = r.Uniform(-mConfiguration.mRadius, mConfiguration.mRadius);
			if (f->mPosition.Length2() <= radius2) {
				break;
			}
		}
		f->mPosition += position;

		mState.mReleaseAmountAccumulator--;
	}
}

void THISCLASS::OnWebotsPhysicsDraw() {/*
	// Get the position
	const dReal *p = dGeomGetPosition(mWebotsInterface.mGeometryID);
	const Point3 position(p[0], p[1], p[2]);

	// Draw the release area
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glLineWidth(1.0);
	glBegin(GL_LINE_LOOP);
	glColor3f(0, 1, 0);
	for (double angle = 0; angle < 2*PI; angle += PI / 6) {
		glVertex3f(position.x + cos(angle)*mConfiguration.mRadius, position.y, position.z - sin(angle)*mConfiguration.mRadius);
	}
	glEnd();
	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);*/
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<FilamentSourceConstant>" << std::endl;
	out << "\t<Radius>" << mConfiguration.mRadius << "</Radius>" << std::endl;
	out << "\t<ReleaseAmount unit=\"filaments/s\">" << mConfiguration.mReleaseAmount << "</ReleaseAmount>" << std::endl;
	out << "\t<FilamentAmount>" << mConfiguration.mFilamentAmount << "</FilamentAmount>" << std::endl;
	out << "\t<FilamentWidth>" << mConfiguration.mFilamentWidth << "</FilamentWidth>" << std::endl;
	out << "\t<FilamentOdorType>" << mConfiguration.mFilamentOdorType << "</FilamentOdorType>" << std::endl;
	out << "</FilamentSourceConstant>" << std::endl;
}
