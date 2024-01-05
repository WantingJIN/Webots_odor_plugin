#ifndef POSITION_H
#define POSITION_H

class Position{ 
    public: 
    
    double x;
    double y;
    double z;

    //constructor : everything set to 0
    Position();
    Position(double px, double py, double pz = 0);
    //copy constructor
    Position(const Position & p);

    void copy_position(const Position to_copy_from);

    bool operator== (const Position& o);
    Position operator+(const Position& o);
   
    void set_to_zero();

    void set_to(double _x, double _y, double _z);

    double distance_from(Position p);

    bool is_in_obstacle(float obs_xmin, float obs_xmax, float obs_ymin, float obs_ymax);

    int is_in_obstacle_list(float ** obs_list, int nbr_obs);
}; 

#endif