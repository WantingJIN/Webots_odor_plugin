// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "OdorModel.h"
#include "Constants.h"
#define THISCLASS OdorModel

THISCLASS::OdorModel(Simulation *sim):
		SimulationInterface(sim), mCutRadius(1) {

	mSimulation->mOdorModel = this;
}

THISCLASS::~OdorModel() {
	if (mSimulation->mOdorModel == this) {
		mSimulation->mOdorModel = 0;
	}
}

double THISCLASS::GetConcentration(const Point3 &point, int odortype) {
	FilamentList *fl = mSimulation->mFilamentList;

	// Sum over all filaments in the selected area
	double concentration = 0;
	double cutradius2 = mCutRadius * mCutRadius;
	for (int i = 0; i < fl->GetCount(); i++) {
		Filament *f = fl->Get(i);
		if ((f->mExists) && (f->mOdorType == odortype)) {
			double dist2 = point.Distance2(f->mPosition);
			if (dist2 <= cutradius2) {
				concentration += f->mAmount / pow(f->mWidth, 3) * exp(-dist2 / pow(f->mWidth, 2));
			}
		}
	}

	return concentration / sqrt(8*pow(PI, 3));
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<OdorModel />" << std::endl;
}

void THISCLASS::WriteConcentration(std::ostream &out, int odortype, Point3 pstart, Point3 pend, Point3 increment) {
	Point3 p;
	for (p.z = pstart.z; p.z < pend.z; p.z += increment.z) {
		for (p.y = pstart.y; p.y < pend.y; p.y += increment.y) {
			for (p.x = pstart.x; p.x < pend.x; p.x += increment.x) {
				double concentration = GetConcentration(p, odortype);
				out << concentration << "\t";
			}
		}
		out << std::endl;
	}
}
