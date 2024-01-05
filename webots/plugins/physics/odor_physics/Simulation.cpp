// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "Simulation.h"
#define THISCLASS Simulation

THISCLASS::Simulation():
		SimulationInterface(this), mSimulationTimeStep(0), mSimulationTime(0), mResultsFolder(), mObstacleList(0), mWindField(0), mFilamentList(0), mFilamentPropagation(0), mOdorModel(0), mFilamentSourceList(0), mSensorList(0) {

}

void THISCLASS::OnSimulationStart() {
	if (! mObstacleList) {
		printf("No obstacle list defined! We have to quit ...\n");
		exit(1);
	}
	if (! mWindField) {
		printf("No wind field defined! We have to quit ...\n");
		exit(1);
	}
	if (! mFilamentList) {
		printf("No filament list defined! We have to quit ...\n");
		exit(1);
	}
	if (! mFilamentPropagation) {
		printf("No filament propagation model defined! We have to quit ...\n");
		exit(1);
	}
	if (! mOdorModel) {
		printf("No odor model defined! We have to quit ...\n");
		exit(1);
	}
	if (! mFilamentSourceList) {
		printf("No filament source list defined! We have to quit ...\n");
		exit(1);
	}
	if (! mSensorList) {
		printf("No sensor list defined! We have to quit ...\n");
		exit(1);
	}

	mObstacleList->OnSimulationStart();
	mWindField->OnSimulationStart();
	mFilamentList->OnSimulationStart();
	mFilamentPropagation->OnSimulationStart();
	mOdorModel->OnSimulationStart();
	mFilamentSourceList->OnSimulationStart();
	mSensorList->OnSimulationStart();
}

void THISCLASS::OnSimulationEnd() {
	mSensorList->OnSimulationEnd();
	mFilamentSourceList->OnSimulationEnd();
	mOdorModel->OnSimulationEnd();
	mFilamentPropagation->OnSimulationEnd();
	mFilamentList->OnSimulationEnd();
	mWindField->OnSimulationEnd();
	mObstacleList->OnSimulationEnd();
}

void THISCLASS::OnSimulationStep() {
	mObstacleList->OnSimulationStep();
	mWindField->OnSimulationStep();
	mFilamentList->OnSimulationStep();
	mFilamentPropagation->OnSimulationStep();
	mOdorModel->OnSimulationStep();
	mFilamentSourceList->OnSimulationStep();
	mSensorList->OnSimulationStep();
}

void THISCLASS::OnWebotsPhysicsDraw() {
	//mObstacleList->OnWebotsPhysicsDraw();
	//mWindField->OnWebotsPhysicsDraw();
	mFilamentList->OnWebotsPhysicsDraw();
	//mFilamentPropagation->OnWebotsPhysicsDraw();
	//mOdorModel->OnWebotsPhysicsDraw();
//mFilamentSourceList->OnWebotsPhysicsDraw();
	//mSensorList->OnWebotsPhysicsDraw();
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<SimulationTime>" << mSimulationTime << "</SimulationTime>" << std::endl;
	out << "<SimulationTimeStep>" << mSimulationTimeStep << "</SimulationTimeStep>" << std::endl;

	mObstacleList->WriteConfiguration(out);
	mWindField->WriteConfiguration(out);
	mFilamentList->WriteConfiguration(out);
	mFilamentPropagation->WriteConfiguration(out);
	mOdorModel->WriteConfiguration(out);
	mFilamentSourceList->WriteConfiguration(out);
	mSensorList->WriteConfiguration(out);
}
