// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <ode/ode.h>
#include <plugins/physics.h>
#include <assert.h>

#include "Simulation.h"
#include "FilamentSourceConstant.h"
#include "WindFieldConstant.h"
#include "WindFieldDynamic.h"
#include "WindFieldStatic.h"
#include "ObstacleList.h"
#include "FilamentList.h"
#include "FilamentSourceList.h"
#include "FilamentSourceConstant.h"
#include "SensorOdor.h"
#include "SensorWind.h"
#include "OdorModel.h"
#include <fstream>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <GL/gl.h>

static const int source_odor_count_max = 16;
// Set the maximal number of the odor, will influence the channel id of the sensor
static const int sensor_odor_count_max = 9;
static const int sensor_wind_count_max = 9;
static bool save_odor_profile = false; // if this is true, then the odor profile will be saved to disk after 20 seconds

Simulation *simulation;

void webots_physics_init() {
//std::cout << "webots_physics_init" << std::endl;
	// Create a simulation object
	simulation = new Simulation();
	simulation->mSimulationTimeStep = 0.032;
	simulation->mSimulationTime = dWebotsGetTime() / 1000.;

	// Store world and space (in case we need that somewhere)
	//simulation->mWebotsInterface.mWorldID = w;
	//simulation->mWebotsInterface.mSpaceID = s;

	// Read parameters from the environment
	char *windsensor_noise_stddev=getenv("WINDSENSOR_NOISE_STDDEV");
	char filament_stddev[10]; //char *filament_stddev=getenv("FILAMENT_STDDEV");
	char filament_growth_gamma[10]; //char *filament_growth_gamma=getenv("FILAMENT_GROWTH_GAMMA");
	char *filament_n=getenv("FILAMENT_N");
	char source_radius[10]; //char *source_radius=getenv("SOURCE_RADIUS");
	char *FReleaseAmount=getenv("FReleaseAmount");
	char *FWindSpeed=getenv("FWindSpeed");
	float wind_x, wind_y;
	float FR_filamentAmount, FR_filamentWidth, FR_releaseAmount;
	
	//FR
	if(access("../../../data/plugin_parameters/FILAMENT_STDDEV.txt", F_OK) != -1 ){
		FILE* sim_param_file = fopen("../../../data/plugin_parameters/FILAMENT_STDDEV.txt", "r");
		int ok = 0;
		ok = fscanf(sim_param_file, "%s", filament_stddev);
		if(ok) printf("filament_stddev : %s\n", filament_stddev);
		fclose(sim_param_file);
	}
	if(access("../../../data/plugin_parameters/FILAMENT_GROWTH_GAMMA.txt", F_OK) != -1 ){
		FILE* sim_param_file = fopen("../../../data/plugin_parameters/FILAMENT_GROWTH_GAMMA.txt", "r");
		int ok = 0;
		ok = fscanf(sim_param_file, "%s", filament_growth_gamma);
		if(ok) printf("filament_growth_gamma : %s\n", filament_growth_gamma);
		fclose(sim_param_file);
	}
	if(access("../../../data/plugin_parameters/SOURCE_RADIUS.txt", F_OK) != -1 ){
		FILE* sim_param_file = fopen("../../../data/plugin_parameters/SOURCE_RADIUS.txt", "r");
		int ok = 0;
		ok = fscanf(sim_param_file, "%s", source_radius);
		if(ok) printf("source_radius : %s\n", source_radius);
		fclose(sim_param_file);
	}
	if(access("../../../data/plugin_parameters/wind_x.txt", F_OK) != -1 ){
		FILE* sim_param_file = fopen("../../../data/plugin_parameters/wind_x.txt", "r");
		int ok = 0;
		ok = fscanf(sim_param_file, "%f", &wind_x);
		if(ok) printf("wind_x : %f\n", wind_x);
		fclose(sim_param_file);
	} else
		wind_x = 0.9;
	if(access("../../../data/plugin_parameters/wind_y.txt", F_OK) != -1 ){
		FILE* sim_param_file = fopen("../../../data/plugin_parameters/wind_y.txt", "r");
		int ok = 0;
		ok = fscanf(sim_param_file, "%f", &wind_y);
		if(ok) printf("wind_y : %f\n", wind_y);
		fclose(sim_param_file);
	} else
		wind_y = 0;
	if(access("../../../data/plugin_parameters/filamentAmount.txt", F_OK) != -1 ){
		FILE* sim_param_file = fopen("../../../data/plugin_parameters/filamentAmount.txt", "r");
		int ok = 0;
		ok = fscanf(sim_param_file, "%f", &FR_filamentAmount);
		if(ok) printf("filamentAmount : %f\n", FR_filamentAmount);
		fclose(sim_param_file);
	} else
		FR_filamentAmount = 8.3e2;
	if(access("../../../data/plugin_parameters/filamentWidth.txt", F_OK) != -1 ){
		FILE* sim_param_file = fopen("../../../data/plugin_parameters/filamentWidth.txt", "r");
		int ok = 0;
		ok = fscanf(sim_param_file, "%f", &FR_filamentWidth);
		if(ok) printf("filamentWidth : %f\n", FR_filamentWidth);
		fclose(sim_param_file);
	} else
		FR_filamentWidth = 0.08;
	if(access("../../../data/plugin_parameters/releaseAmount.txt", F_OK) != -1 ){
		FILE* sim_param_file = fopen("../../../data/plugin_parameters/releaseAmount.txt", "r");
		int ok = 0;
		ok = fscanf(sim_param_file, "%f", &FR_releaseAmount);
		if(ok) printf("releaseAmount : %f\n", FR_releaseAmount);
		fclose(sim_param_file);
	} else
		FR_releaseAmount = 20;
	//FR end

	printf("_______________________%s %s\n", FReleaseAmount, FWindSpeed);
	printf("_______________________%s %s %s\n", filament_stddev, filament_growth_gamma, source_radius);
    
    char *save_profile=getenv("SAVE_ODOR_PROFILE");
    save_odor_profile = (NULL != save_profile);

	// Prepare the results folder
	char buffer[1024];
	if (getcwd(buffer, 1024) == 0) {
		std::cout << "Failed to get CWD - we won't be able write any results." << std::endl;
	}
	simulation->mResultsFolder = std::string(buffer);
	simulation->mResultsFolder += "/results";
	mkdir(simulation->mResultsFolder.c_str(), 0755);

	// Add a filament list
	// The number of filaments should be adjusted such that the filaments disappear roughly at the border of the simulation area.
	// WEBOTS 2019 UPDATE
	// 200 is the number of filaments. Should be the same as the number in the webots supervisor
	new FilamentList(simulation, (filament_n ? strtol(filament_n, 0, 0) : 2400));
// Fa
	// Add a constant wind field
	new ObstacleList(simulation);
	WindFieldConstant *wf = new WindFieldConstant(simulation);
	wf->SetWindSpeed(Point3(-(FWindSpeed ? strtof(FWindSpeed, 0) : 0.9), 0.0f, 0.0f)); // XXX: constant? // (X,Z,Y) !!!
	wf->SetWindSpeed(Point3(-wind_x, -wind_y, 0.0f));

	// Add a static wind field
	// new ObstacleList(simulation);
	// WindFieldStatic *wf = new WindFieldStatic(simulation);
	// if(access("../../../data/plugin_parameters/wind_map.txt", F_OK) != -1 ){
	// 	wf->SetFile("../../../data/plugin_parameters/wind_map.txt");
	// 	wf->WriteConfiguration(std::cout);
	// }else
	// 	printf("NO WIND MAP WAS FOUND!!!\n");

	/*
    // Add a dynamic wind field
    new ObstacleList(simulation);
    WindFieldDynamic *wf = new WindFieldDynamic(simulation);
    wf->SetFolder("/home/ercolani/Documents/OpenFoam/plugin/OpenFoam_to_test/37_NoSlip");
	// "/home/rahbar/OpenFOAM/rahbar-v3.0+/run/PitzDaily_newTest_Obstacle_diffY_newBoundary_newObstacle_morePoint_lowSpeed"
	// "/disal/rahbar/OpenFOAM_data/Wind0.1_Mesh15_ObstacleBig_Timestep0.16"
	*/
// Fa


	// Add a filament propagation model
	FilamentPropagation *fp = new FilamentPropagation(simulation);
	fp->mConfiguration.mStdDev = (filament_stddev ? strtof(filament_stddev, 0) : 0.2); //0.02
	fp->mConfiguration.mFilamentGrowthGamma = (filament_growth_gamma ? strtof(filament_growth_gamma, 0) : 4e-7);

	//printf("Running with stddev=%f gamma=%f\n", fp->mConfiguration.mStdDev, fp->mConfiguration.mFilamentGrowthGamma);

	// Add an odor concentration model
	OdorModel *om = new OdorModel(simulation);
	om->mCutRadius = 1;

	// Add a list of filament source and the sensors
	FilamentSourceList *filamentsourcelist = new FilamentSourceList(simulation);
	SensorList *sensorlist = new SensorList(simulation);

	// Add odor sources
	for (int i = 0; i < source_odor_count_max; i++) {
		char buffer[32];
		sprintf(buffer, "SOURCE_ODOR_%d", i);
		dGeomID geometry_id = dWebotsGetGeomFromDEF(buffer);
		if (geometry_id > 0) {
			FilamentSourceConstant *fs = new FilamentSourceConstant(simulation);
			fs->mWebotsInterface.mGeometryID = geometry_id;
			fs->mConfiguration.mRadius = (source_radius ? strtof(source_radius, 0) : 0.02);
			fs->mConfiguration.mFilamentAmount = FR_filamentAmount; //8.3e2;  //e9
			fs->mConfiguration.mFilamentWidth = FR_filamentWidth; //0.08;
			fs->mConfiguration.mFilamentOdorType = 0;
			fs->mConfiguration.mReleaseAmount = FR_releaseAmount; //20;//(FReleaseAmount ? strtof(FReleaseAmount, 0) : 20);//20; //100
			filamentsourcelist->AddFilamentSource(fs);
		}
	}

	// Add odor sensors
	for (int i = 0; i < sensor_odor_count_max; i++) {
		char buffer[32];
		sprintf(buffer, "SENSOR_ODOR_%d", i);
		dGeomID geometry_id = dWebotsGetGeomFromDEF(buffer);
		if (geometry_id > 0) {
			//printf("odor sensor found %d\n", i);
			SensorOdor *s = new SensorOdor(simulation);
			s->mWebotsInterface.mGeometryID = geometry_id;
			s->mWebotsInterface.mChannel = i;
			s->mWebotsInterface.mLogFile.open((simulation->mResultsFolder + "/" + buffer).c_str());
			s->mConfiguration.mOdorType = 0;
			s->mConfiguration.mNoiseStdDev = 0;
			s->mConfiguration.mRunningAverageFactor = 0.0;
			sensorlist->AddSensor(s);
		}
	}

	// Add wind sensors
	for (int i = 0; i < sensor_wind_count_max; i++) {
		char buffer[32];
		sprintf(buffer, "SENSOR_WIND_%d", i);
		dGeomID geometry_id = dWebotsGetGeomFromDEF(buffer);
		if (geometry_id > 0) {
			//printf("new wind sensor found %d\n", i);
			SensorWind *s = new SensorWind(simulation);
			s->mWebotsInterface.mGeometryID = geometry_id;
			s->mWebotsInterface.mChannel = i + sensor_odor_count_max;
			s->mWebotsInterface.mLogFile.open((simulation->mResultsFolder + "/" + buffer).c_str());
			s->mConfiguration.mNoiseStdDev = (windsensor_noise_stddev ? strtod(windsensor_noise_stddev, 0) : 0.01);
			s->mConfiguration.mRunningAverageFactor = 0.0;
			sensorlist->AddSensor(s);
		}
	}

	// Initialize everything
	simulation->OnSimulationStart();

	// Write configuration
	std::ofstream file((simulation->mResultsFolder + "/configuration.xml").c_str());
	file << "<?xml version=\"1.0\" ?>" << std::endl;
	file << std::endl;
	file << "<!-- This XML file is written by the odor_physics plugin at the beginning of each simulation with the configuration. It cannot be read by the the odor_physics plugin, but two such files can be compared to check for configuration differences. -->" << std::endl;
	file << std::endl;
	file << "<OdorPhysics>" << std::endl;
	simulation->WriteConfiguration(file);
	file << "</OdorPhysics>" << std::endl;

//std::cout << "webots_physics_init end" << std::endl;
}

void webots_physics_step() {
	// Run one timestep of the simulation
	double simulation_time = dWebotsGetTime() / 1000.;
	simulation->mSimulationTimeStep = simulation_time - simulation->mSimulationTime;
	simulation->mSimulationTime = simulation_time;
	//std::cout << simulation->mSimulationTime << std::endl;
	simulation->OnSimulationStep();

	// Store an odor profile after 20 seconds
  if (save_odor_profile) {
    if ((simulation->mSimulationTime - 20. > 0) && (simulation->mSimulationTime - 20. < 0.064)) {
      std::cout << "Writing odor profile" << std::endl;
      std::ofstream file((simulation->mResultsFolder+"/concentration20").c_str());
      // Startpoint, endpoint, resolution
      //simulation->mOdorModel->WriteConcentration(file, 0, Point3(0, 0, -2), Point3(16, 1, 2), Point3(0.01, 2, 0.01));
      //simulation->mOdorModel->WriteConcentration(file, 0, Point3(0, 0, -2), Point3(20, 1, 2), Point3(0.01, 2, 0.01));
      simulation->mOdorModel->WriteConcentration(file, 0, Point3(-14, 0, -2), Point3(6, 1, 2), Point3(0.01, 2, 0.01));
      exit(0);
    }
  }

}


void webots_physics_step_end(){
	simulation->OnWebotsPhysicsDraw();
}


int webots_physics_collide(dGeomID g1, dGeomID g2) {
//std::cout << "webots_physics_collide" << std::endl;
	return 0;	// collision not handled, otherwise should return 1
}

void webots_physics_cleanup() {
//std::cout << "webots_physics_cleanup" << std::endl;
	// Cleanup everything
	simulation->OnSimulationEnd();
}

/*
void webots_physics_draw(int pass, const char *view) {
std::cout << "webots_physics_draw" << std::endl;
	// Draw the world
	simulation->OnWebotsPhysicsDraw();
//std::cout << "webots_physics_draw end" << std::endl;
	// Draw coordinates
	glDisable(GL_LIGHTING);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glColor3f(0, 0, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glEnd();
	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
}*/
