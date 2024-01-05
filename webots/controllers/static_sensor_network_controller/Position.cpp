#include <math.h>
//#include "nvwa/debug_new.h"
#include "Position.h"

Position::Position(){
	x = 0;
	y = 0;
	z = 0;
}

Position::Position(double px, double py, double pz){
	x = px;
	y = py;
	z = pz;
}

Position::Position(const Position & to_copy_from){
	x = to_copy_from.x;
	y = to_copy_from.y;
	z = to_copy_from.z;
}

bool Position::operator== (const Position& o){
	return (o.x == x && o.y == y && o.z == z); 
}

Position Position::operator+(const Position& o){
	return Position(o.x + x, o.y + y, o.z + z); 
}

void Position::copy_position(const Position to_copy_from){
	x = to_copy_from.x;
	y = to_copy_from.y;
	z = to_copy_from.z;
}

void Position::set_to_zero(){
	x = 0;
	y = 0;
	z = 0;
}

void Position::set_to(double _x, double _y, double _z){
	x = _x;
	y = _y;
	z = _z;
}

double Position::distance_from(Position p){
	return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2) + pow(z - p.z,2));
}

bool Position::is_in_obstacle(float obs_xmin, float obs_xmax, float obs_ymin, float obs_ymax){
	if(x > obs_xmin && x < obs_xmax && y > obs_ymin && y < obs_ymax)
		return true;
	else
		return false;
}

int Position::is_in_obstacle_list(float ** obs_list, int nbr_obs){
	// gets an obstacle list and number of obstacles as input
	// returns the index of the obstacle that the position is in 
	// if the position is not in any obstacles, then -1 is returned
	for(int i = 0; i < nbr_obs; i++){
		if(is_in_obstacle(obs_list[i][0], obs_list[i][2], obs_list[i][1], obs_list[i][3]))
			return i;
	}
	return -1;
}