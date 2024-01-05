/*
 * File:         static_sensor_network_controller.c
 * Description:  This is a supervisor node to place some static sensors and log their values
 * Author:       Wanting Jin - Jan 2020
 * Note:         
 *               
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>
#include <sys/stat.h> // mkdir
#include <sys/time.h>
#include <vector>
#include <unistd.h> //access()
#include "gasMap2D.h"

#include "SampleBuffer.h"
#include "Position.h"
#include "tinyxml2.h"

#define PI 3.14159265359
#define TIME_STEP           64 //adjusts the speed (ms)
#define SENSOR_NUM_MAX      9
#define ODOR_FILTER_LENGTH  150 //~10sec //number of measurements before averaging
#define ITERATIONS          1
#define RANDOM0_GRID1       1
#define RANDOM_ITERATIONS   0

#define XMIN 2.75
#define XMAX 12.7418
#define XSTEP 0.1586
#define XNUM 64

#define YMIN 0.01
#define YMAX 3.9916
#define YSTEP 0.0632
#define YNUM 64

using namespace tinyxml2;

WbFieldRef root_children_field;
WbDeviceTag odor_tag[SENSOR_NUM_MAX];
WbDeviceTag wind_tag[SENSOR_NUM_MAX];
vector<SampleBuffer> sampleBuffer;
int sensor_number = 0;
vector<Position> sensor_positions;
vector<double> sensor_measurements;
//double sensor_position[SENSOR_NUM_MAX][3] = {{0}};
Position source_position;

int map_nbr = 0;
int nbr_obstacle=0;
float **obstacles = NULL;

// log files names
char folder_name[256];
char measurements_logfilename[300];
char gas_distribution_map_logfilename[300];
char config_logfilename[300];
FILE* measurements_logfile;
FILE* gas_distribution_map_logfile;
FILE* config_logfile;

// intialize the gas distribution map
//param: sigma, gamma, scaling_param, wind_speed, rco
GasMap2D gas_map(0.2, 2, 1, 0.75, 0.3);

void change_number_of_sensors();
void read_sensor_positions_from_file();
void place_sensor(int sensor_ID, Position position);
void place_sensor_to_zero(int sensor_ID);
void wind_read(int i, double* intensity, double* angle);
bool in_obstacle(Position pos);
void setup_log_folder();
void log_measurement();
long long current_timestamp();

void init(){
    wb_robot_init();
    srand((long)time(NULL));
    setup_log_folder();

    //Get static sensor network
    WbNodeRef root_node = wb_supervisor_node_get_self();
  
    //Define position of new node inside the children field
    root_children_field = wb_supervisor_node_get_field(root_node, "children");
  
    //change_number_of_sensors();
    read_sensor_positions_from_file();
    //create new sensor nodes
    for (int i = 0; i < sensor_number; i++){
        //apply the position to both the odor sensor and wind sensor of ID i
        place_sensor(i, sensor_positions[i]);
        //activate the sensors
        //odor
        char tag_name[256];
        sprintf(tag_name,"sensor_odor_%d",i);
        odor_tag[i] = wb_robot_get_device(tag_name);
        wb_receiver_enable(odor_tag[i], TIME_STEP);
        //wind
        sprintf(tag_name,"sensor_wind_%d",i);
        wind_tag[i] = wb_robot_get_device(tag_name);
        wb_receiver_enable(wind_tag[i],TIME_STEP);
        //set buffer length for all sensors
        SampleBuffer sample;
        sample.set_buffer_length(ODOR_FILTER_LENGTH);
        sampleBuffer.push_back(sample);
    }
    for (int i = sensor_number; i < SENSOR_NUM_MAX; i++){
        //set other sensors position to 0
        place_sensor_to_zero(i);
    }
    //read and save the obstacle list
    // open file
	std::ifstream obstacle_file("../../data/obstacle_list.txt");
	// first line is the number of obstacles
	obstacle_file >> nbr_obstacle;
	printf("There is %d obsatcle in total:\n",nbr_obstacle);
	// allocate memory for obstacle list array
	obstacles = new float*[nbr_obstacle];
	//read all obstacles now
	for(int i = 0; i < nbr_obstacle; i++){
		obstacles[i] = new float[4];
		obstacle_file >> obstacles[i][0] >> obstacles[i][1] >> obstacles[i][2] >> obstacles[i][3];
	}

	for(int i = 0; i < nbr_obstacle; i++){
		printf("obstacle %d: %f %f %f %f\n", i, obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]);
	}

    // set the source position randomly (on upperwind half on x)
    do{
        source_position.x = ( (rand()/(double)RAND_MAX) * (XMAX-XMIN)/2 ) + XMIN + (XMAX-XMIN)/2;
        source_position.y = ( (rand()/(double)RAND_MAX) * (YMAX-YMIN) ) + YMIN;
        source_position.z = 0.1;
    } while(in_obstacle(source_position));
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def("SOURCE_ODOR_0"),"translation"), (double*)&source_position);
    // load map
    //read simulation parameters
	FILE* sim_param_file;
	int ok = 0;
    if(access("../../data/map_nbr.txt", F_OK) != -1 ){
		sim_param_file = fopen("../../data/map_nbr.txt", "r");
		ok = fscanf(sim_param_file, "%d", &map_nbr);
		if(ok) printf("scenario : %d\n", map_nbr);
		fclose(sim_param_file);
	}
    // log config
    config_logfile = fopen(config_logfilename, "w");
    fprintf(config_logfile, "source_pos %f %f %f\n", source_position.x, source_position.y, source_position.z);
    fprintf(config_logfile, "sensor_nbr %d\n", sensor_number);
    fprintf(config_logfile, "map_nbr %d\n", map_nbr);
    fclose(config_logfile);
}
long long current_timestamp(){
	struct timeval te; 
	gettimeofday(&te, NULL); // get current time
	long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
	// printf("milliseconds: %lld\n", milliseconds);
	return milliseconds;
}

void setup_log_folder(){
    //log files names
	char* pPath = getenv("WB_WORKING_DIR");
    if (pPath!=NULL){
    	sprintf(folder_name, "%s/%lld",  pPath, current_timestamp());
    	mkdir(folder_name, 0755);
    }else{
    	sprintf(folder_name, "../../../Results/current"); //create a new folder for log files
    	mkdir(folder_name, 0755);
    }
    sprintf(measurements_logfilename, "%s/measurements.csv", folder_name);
    sprintf(gas_distribution_map_logfilename, "%s/gas_distribution_map.csv", folder_name);
    sprintf(config_logfilename, "%s/config_log.csv", folder_name);

    measurements_logfile = fopen(measurements_logfilename, "w");
    fclose(measurements_logfile);
    gas_distribution_map_logfile = fopen(gas_distribution_map_logfilename, "w");
    fclose(gas_distribution_map_logfile);
    config_logfile = fopen(config_logfilename, "w");
    fclose(config_logfile);
}

bool in_obstacle(Position pos){
    // returns true if position in inside obstacle
    float x = pos.x;
    float y = pos.y;
    //obstacles (traning_map1, 4 rectangles)
    // if(x >= 11.5 && x <= 12 && y >= 0 && y <= 1.5)
    //     return true;
    // if(x >= 9 && x <= 12 && y >= 1 && y <= 1.5)
    //     return true;
    // if(x >= 5 && x <= 8 && y >= 2.5 && y <= 3)
    //     return true;
    // if(x >= 5 && x <= 5.5 && y >= 2.5 && y <= 4)
    //     return true;
    for(int i = 0; i < nbr_obstacle; i++){
        if(x >= obstacles[i][0] && y >= obstacles[i][1] && x <= obstacles[i][2] && y <= obstacles[i][3])
            return true;
    }
    // if none of the above, then out of obstacle
    return false;
}

void read_sensor_positions_from_file(){
    int res = 0;
    FILE *pFile = fopen("../../data/sensor_position.txt", "r");
    //read the number of sensors
    res = fscanf(pFile, "%d", &sensor_number);
    if(res == 0)
        printf("/!\\ no sensor number found!!! \n");
    //read the positions
    for(int i = 0; i < sensor_number; i++){
        Position sensor_pos;
        res = fscanf(pFile, "%lf %lf", &(sensor_pos.x), &(sensor_pos.y));
        sensor_pos.z = 0.1;
        sensor_positions.push_back(sensor_pos);
        printf("sensor %d position %f %f \n", i, sensor_pos.x, sensor_pos.y);
    }
}

void change_number_of_sensors(){
    //open txt file and make the necessary modifications, then copy paste in the world file
    FILE * wbo_file = fopen("../../protos/sensor_odor_receiver.wbo", "w");
    fprintf(wbo_file, "#VRML_OBJ R2021b utf8\n");
    for (int i = 0; i < SENSOR_NUM_MAX; i++){
        fprintf(wbo_file, "DEF SENSOR_ODOR_%d Receiver {\n\
        translation %f %f 0.1\n\
        rotation 1 0 0 -1.5708\n\
        children [\n\
            Shape {\n\
                appearance Appearance {\n\
                    material Material {\n\
                         ambientIntensity 0.5\n\
                         diffuseColor 0.7 1 0\n\
                         emissiveColor 1 1 1\n\
                    }\n\
                    texture ImageTexture {\n\
                        url [\n\
                            \"../pic/sensor_node3.png\"\n\
                        ]\n\
                        repeatS FALSE\n\
                    }\n\
                }\n\
                geometry DEF CYLINDER_ODOR Cylinder {\n\
                    height 0.04\n\
                    radius 0.1\n\
                    subdivision 13\n\
                }\n\
            }\n\
        ]\n\
        name \"sensor_odor_%d\"\n\
        boundingObject USE CYLINDER_ODOR\n\
        physics Physics {\n\
        }\n\
        channel %d\n\
        }\n", i, 3.0+(rand()/(double)RAND_MAX)*10.0, (rand()/(double)RAND_MAX)*4.0, i, i);
    }
    fclose(wbo_file);

    //open txt file and make the necessary modifications
    FILE * wbo_file_w = fopen("../../protos/sensor_wind_receiver.wbo", "w");
    fprintf(wbo_file_w, "#VRML_OBJ R2021b utf8\n");
    for (int i = 0; i < SENSOR_NUM_MAX; i++){
        fprintf(wbo_file_w, "DEF SENSOR_WIND_%d Receiver {\n\
        translation %f %f 0.2\n\
        rotation 1 0 0 -1.5708\n\
        children [\n\
            Shape {\n\
                appearance Appearance {\n\
                    material Material {\n\
                        diffuseColor 1 0 0\n\
                        emissiveColor 1 1 1\n\
                    }\n\
                    texture ImageTexture {\n\
                        url [\n\
                            \"../pic/wind_sensor6.png\"\n\
                        ]\n\
                        repeatS FALSE\n\
                    }\n\
                }\n\
                geometry DEF CYLINDER_WIND Cylinder {\n\
                    height 0.04\n\
                    radius 0.1\n\
                    subdivision 13\n\
                }\n\
            }\n\
        ]\n\
        name \"sensor_wind_%d\"\n\
        boundingObject USE CYLINDER_WIND\n\
        physics Physics {\n\
        }\n\
        channel %d\n\
        }\n", i, 3.0+(rand()/(double)RAND_MAX)*10.0, (rand()/(double)RAND_MAX)*4.0, i, i+SENSOR_NUM_MAX);
    }
    fclose(wbo_file_w);

    //add the node to the scene tree
    //wb_supervisor_field_import_mf_node(root_children_field, -1, "../../protos/sensor_odor_receiver.wbo");
    
    //exit to start again
    printf("number of sensors changed successfully, now close and run again the simulation.\n");
    exit(0);
}

double odor_read(int i){
    double odor_sample = 0;
    while (wb_receiver_get_queue_length(odor_tag[i]) > 0) {
        assert(wb_receiver_get_data_size(odor_tag[i]) == sizeof(double));
        odor_sample = *(const double *)wb_receiver_get_data(odor_tag[i]);
        wb_receiver_next_packet(odor_tag[i]);
    }
    //printf("odor measurement : %f \n",odor_measurement);
    
    //if sensor is placed in obstacle, we send only 0
    if(in_obstacle(sensor_positions[i]))
        return 0;
    else
        return odor_sample;
}

void wind_read(int i, double* intensity, double* angle){
    struct vect3D{
        double x;
        double y;
        double z;
    };
    struct vect3D v = {0};
    while (wb_receiver_get_queue_length(wind_tag[i]) > 0) {
        assert(wb_receiver_get_data_size(wind_tag[i]) == sizeof(struct vect3D));
        v = *(const struct vect3D*)wb_receiver_get_data(wind_tag[i]);
        wb_receiver_next_packet(wind_tag[i]);
    }
    //convert the 3D vector from the sensor to intensity and heading
    *intensity = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    *angle = atan2(v.y, v.x); //randians: when calculating average value, it use the radian and convert to degree when loging
    //printf("recived wind data %f %f %f\n", v.x, v.y, v.z);
    //printf("angle is %f\n", *angle);
    //keep the 3D vector
    //*ux = v.x; 
    //*uy = v.y;
    //*uz = v.z;

    //if sensor is placed in obstacle, we send only 0
    if(in_obstacle(sensor_positions[i])){
        *intensity = 0;
        *angle = 0;
    }
}

void place_sensor(int sensor_ID, Position sensor_pos){
    //wind sensor
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID),
        "translation"), (double*)&sensor_pos);
    //odor sensor
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID+SENSOR_NUM_MAX),
        "translation"), (double*)&sensor_pos);
    //correct orientation for wind sensors /!\ determines the origin for wind angle! should match with Khepera IV
    const double rotation_wind_sensors[] = {0, 0.70710528118436, -0.707108281185553, -3.1415923071795864};
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID),
        "rotation"), rotation_wind_sensors);
}
 
void place_sensor_to_zero(int sensor_ID){
     //wind sensor
    double origin_pos[3] = {0};
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID),
        "translation"), origin_pos);
    //odor sensor
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID+SENSOR_NUM_MAX),
        "translation"), origin_pos);
    //correct orientation for wind sensors /!\ determines the origin for wind angle! should match with Khepera IV
    const double rotation_wind_sensors[] = {0, 0.70710528118436, -0.707108281185553, -3.1415923071795864};
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID),
        "rotation"), rotation_wind_sensors);
}
void log_measurement(){
    //read plugin's info using tinyxml2 library
    float Q = 0;
    XMLDocument doc;
    doc.LoadFile("../../plugins/physics/odor_physics/results/configuration.xml"); //this file is generated by the odor plugin
    const char* Q_line = doc.FirstChildElement("OdorPhysics")->FirstChildElement("FilamentSourceList")->FirstChildElement("FilamentSourceConstant")->FirstChildElement("ReleaseAmount")->GetText();
    sscanf(Q_line, "%f", &Q);
    printf("measurement path is %s\n", measurements_logfilename);
    measurements_logfile = fopen(measurements_logfilename, "w");
    for(int i=0; i < sensor_number; i++){
        //take the average
        sampleBuffer[i].average_all();
        //log in the data_base
        printf("average wind angle is: %f\n", sampleBuffer[i].average_wind_angle);
        printf("average concentration is: %f\n", sampleBuffer[i].average_concentration);
        fprintf(measurements_logfile, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
            source_position.x, source_position.y, source_position.z, Q,                  // Sx, Sy, Sz, Q,
            sampleBuffer[i].average_wind_intensity, sampleBuffer[i].average_wind_angle,     // wind_intensity, wind angle,
            sensor_positions[i].x, sensor_positions[i].y, sensor_positions[i].z,            // px, py, pz,
            sampleBuffer[i].average_concentration, sampleBuffer[i].stdDev_concentration);   // C, stdC
    }
    fclose(measurements_logfile);
}

int main() {
    int i = 0;
    int n_samples = 0;
    int wait = 200;

    init();
    printf("Finish init\n");
    
    //Get the sensor values and write them in a file

    while(wait > 0){
        wait--; //wait for the plume to be established
        wb_robot_step(TIME_STEP);
        if(wait == 1) printf("done waiting!\n");
    }
    //SAMPLE
    while (n_samples < ODOR_FILTER_LENGTH){
        for(i=0; i < sensor_number; i++){
            //continue measuring odor concentration
            sampleBuffer[i].instant_concentration = odor_read(i);
            //printf("odor sensor %d -> %f \n", i, sampleBuffer[i].instant_concentration);
            //wind 
            wind_read(i, &sampleBuffer[i].instant_wind_intensity, &sampleBuffer[i].instant_wind_angle);
            //printf("wind sensor %d -> %f %f\n", i, sampleBuffer[i].instant_wind_intensity, sampleBuffer[i].instant_wind_angle);
            sampleBuffer[i].add_all_to_buffer();
        }
        n_samples++;
        wb_robot_step(TIME_STEP);
    }
    //LOG
    gas_map.buildGasDistributionMap(sensor_positions, sampleBuffer);
    printf("finish building gas map\n");
    log_measurement();
    printf("finish logging measurements\n");
    gas_map.logGasMap(gas_distribution_map_logfilename);
    gas_map.printGasMap();
    printf("finish logging gas distribution map\n");
    printf("Data collection over\n");
            
    //NEXT
    wb_robot_step(TIME_STEP);
    sprintf(folder_name, "../../../Results/%lld", current_timestamp());
    rename("../../../Results/current", folder_name);

    wb_supervisor_simulation_quit(EXIT_SUCCESS);
    wb_robot_cleanup();
    return 0;
}
