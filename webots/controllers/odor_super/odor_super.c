/*
 * File:         odor_super.c
 * Description:  This is a supervisor node to render the odor_physics_plugin output
 * Author:       Chiara Ercolani 4-4-2019
 * Note:         This code is made for Webots 2019a since the physics plugin do not support Open GL anymore
 *               
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>

#define PARTICLE_NUM 800

WbDeviceTag receiver; // Handle for the receiver node for odor information from physics plugin


int main() {
	wb_robot_init();
	int time_step = wb_robot_get_basic_time_step();
	if (time_step == 0)
		time_step = 1;
		
	//get receiver  
	receiver = wb_robot_get_device("odor_receiver");
	wb_receiver_enable(receiver,64);
	
	//Get odor supervisor node  
	WbNodeRef root_node = wb_supervisor_node_get_self();
	
	//Define position of new node inside the children field
	WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
	
	//create new nodes according to the Transform.wbo file
	// One node per particle
	for (int i=0; i<PARTICLE_NUM; i++)
		wb_supervisor_field_import_mf_node(root_children_field, i, "children/Transform_hex.wbo");
	
	for (;;){
		//Check queue of receiver
		if(wb_receiver_get_queue_length(receiver) > 0){
			const double *physics_buffer = (double*) wb_receiver_get_data(receiver);
			//loop on the buffer to get the position of all the particles (the ones that are not moving are initialized to zero)
			for (int i=0; i<PARTICLE_NUM; i++){
				//Update translation field of particles with the appropriate position
				if(physics_buffer[i*4] > 0 && (int)physics_buffer[i*4] <= PARTICLE_NUM){
					wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field,(int)physics_buffer[i*4]),"translation"), physics_buffer+i*4+1);
					//printf("particle %d to %f %f %f\n", (int)physics_buffer[i*4], physics_buffer[i*4+1], physics_buffer[i*4+2], physics_buffer[i*4+3]);
				}
			}
			wb_receiver_next_packet(receiver);
		}
		wb_robot_step(time_step);
	}
	return 0;
}
