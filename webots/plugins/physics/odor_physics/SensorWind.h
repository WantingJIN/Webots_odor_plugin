// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classSensorWind
#define classSensorWind

class SensorWind;

#include "Sensor.h"
#include "Point3.h"
#include <fstream>
#include <ode/ode.h>

//! A wind sensor.
class SensorWind: public Sensor {

public:
	//! Webots interface
	struct {
		dGeomID mGeometryID;			//!< Geometry ID, which allows us to get the position.
		int mChannel;					//!< Sender channel, which we will write the concentration value to.
		std::ofstream mLogFile;			//!< File to which we will write all sensor values.
	} mWebotsInterface;

	//! Sensor configuration.
	struct {
		double mNoiseStdDev;			//!< Standard deviation of the noise.
		double mRunningAverageFactor;	//!< Factor for running average (over time, i.e. at each invocation of SimulationStep this factor is used).
	} mConfiguration;

	//! Sensor state.
	struct {
		Point3 mWind;					//!< The measured wind.
	} mState;

	//! Constructor
	SensorWind(Simulation *sim);
	//! Destructor
	~SensorWind();

	// Sensor methods
	void OnSimulationStart();
	void OnSimulationEnd();
	void OnSimulationStep();
	void OnWebotsPhysicsDraw();
	void WriteConfiguration(std::ostream &out);

	// Rotate vector v by quaternion q
	inline void qRotateVector(const dQuaternion q, Point3 &v) {
		double v1 = v.x;
		double v2 = v.y;
		double v3 = v.z;
		double t2 =   q[0] * q[1];
		double t3 =   q[0] * q[2];
		double t4 =   q[0] * q[3];
		double t5 =  -q[1] * q[1];
		double t6 =   q[1] * q[2];
		double t7 =   q[1] * q[3];
		double t8 =  -q[2] * q[2];
		double t9 =   q[2] * q[3];
		double t10 = -q[3] * q[3];
		v.x = 2.0 * ((t8 + t10) * v1 + (t6 -  t4) * v2 + (t3 + t7) * v3) + v1;
		v.y = 2.0 * ((t4 +  t6) * v1 + (t5 + t10) * v2 + (t9 - t2) * v3) + v2;
		v.z = 2.0 * ((t7 -  t3) * v1 + (t2 +  t9) * v2 + (t5 + t8) * v3) + v3;
	}
};

#endif
