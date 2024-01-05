// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "FilamentPropagation.h"
#include "Random.h"
#define	THISCLASS FilamentPropagation

using namespace std;

THISCLASS::FilamentPropagation(Simulation *sim):
		SimulationInterface(sim) {

	mSimulation->mFilamentPropagation = this;
	mConfiguration.mStdDev = 0;
	mConfiguration.mFilamentGrowthGamma = 0;
}

THISCLASS::~FilamentPropagation() {
	if (mSimulation->mFilamentPropagation == this) {
		mSimulation->mFilamentPropagation = 0;
	}
}

void THISCLASS::OnSimulationStart() {
}

void THISCLASS::OnSimulationEnd() {
}

void THISCLASS::OnSimulationStep() {
	WindField *wf = mSimulation->mWindField;
	FilamentList *fl = mSimulation->mFilamentList;
	ObstacleList *of = mSimulation->mObstacleList;
	double simstep = mSimulation->mSimulationTimeStep;
	double stddev = mConfiguration.mStdDev * simstep;
	int count = fl->GetCount();
	Point3 currentWind;
	
	//Fa open a file to write informations abour filaments to be checked
	//ofstream myfile;
	//myfile.open("/home/rahbar/Desktop/FaezehRahbar/Program/filament_Thomas/Webots/plugins/physics/odor_physics/results/filaments.txt", ios::app);

	Random r;
	for (int i = 0; i < count; i++) {
		Filament *f = fl->Get(i);
		if (f->mExists) {
			f->mPrevPosition = f->mPosition;
			
			//check for error of wind
			currentWind = wf->GetWindSpeed(f->mPosition);
			if (currentWind == Point3(-100,-100,-100)){
				/*std::cout << "filament " << f->mID << " got removed (position: " 
							<< f->mPosition << ")" << std::endl;*/
				fl->RemoveFilament(f->mID);
				continue;
			}
			
			// Advection
			Point3 newpos = f->mPosition + currentWind * simstep;

            //if(i==0) printf("old:(%f %f %f) new:(%f %f %f) ws:(%f %f %f)\n",f->mPosition.x,f->mPosition.y,f->mPosition.z,newpos.x, newpos.y, newpos.z, wf->GetWindSpeed(f->mPosition).x, wf->GetWindSpeed(f->mPosition).y, wf->GetWindSpeed(f->mPosition).z);
            /*if(i==1){ //Fa
            std::cout << "old:("<< f->mPosition.x << f->mPosition.y << f->mPosition.z << ") new:(" << newpos.x << newpos.y << newpos.z << ") ws:(" << wf->GetWindSpeed(f->mPosition).x << wf->GetWindSpeed(f->mPosition).y << wf->GetWindSpeed(f->mPosition).z << std::endl;
              }*/

			// Stochastic process (vmi)
			newpos.x += r.Normal(0, stddev);
			newpos.y += r.Normal(0, stddev);
			newpos.z += r.Normal(0, stddev);

			// write the position and wind speed of all the filaments in a file (Fa)
			/*if (myfile.is_open()){} 
			else{std::cout << "error while opening the file"<< std::endl;}
			myfile << "filament " << i << "\n";
			myfile << "old:("<< f->mPosition.x << " "<< f->mPosition.y << " "<< f->mPosition.z 
					<< ") new:(" << newpos.x << " "<< newpos.y << " "<< newpos.z << ") ws:(" 
					<< wf->GetWindSpeed(f->mPosition).x << " "<< wf->GetWindSpeed(f->mPosition).y
					<< " "<< wf->GetWindSpeed(f->mPosition).z << ")\n";*/

			// Simple way of dealing with obstacles: 
			// if the new filament position is inside an obstacle, simply don't move
			if (of->GetObstacle(newpos) == 0) {
				f->mPosition = newpos;
			}else
				std::cout << "filament " << f->mID << " in an obstacle (position: " 
						  << f->mPosition << ")" << std::endl;
			// Filament growth
			f->mWidth += 0.5 * mConfiguration.mFilamentGrowthGamma / f->mWidth;
			
		}
	}
	//myfile.close();
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<FilamentPropagation>" << std::endl;
	out << "\t<StdDev>" << mConfiguration.mStdDev << "</StdDev>" << std::endl;
	out << "\t<FilamentGrowthGamma>" << mConfiguration.mFilamentGrowthGamma << "</FilamentGrowthGamma>" << std::endl;
	out << "</FilamentPropagation>" << std::endl;
}
