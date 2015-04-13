#ifndef ROBOT_FOOTPRINT_H
#define ROBOT_FOOTPRINT_H

#include "motion_planning_structures.h"

class robot_footprint{
public:
	//robot_footprint(std::vector<cart_point> _foot_print);
	//robot_footprint(cart_point corner1, cart_point corner2, cart_point corner3, cart_point corner4);//relative to turning center
	//robot_footprint(cart_point origin, double length=LENGTH, double width=WIDTH); //origin is relative to right rear corner to match standard ros coordinate frame
	robot_footprint(double length=LENGTH, double width=WIDTH, double origin_x=ORIGIN_X, 
		double origin_y=ORIGIN_Y, double grid_size=GRID_SIZE); 

	std::vector<cart_point> foot_print;

	void print_footprint();
	void print_footprint_at(double theta);
	std::vector<cart_point> foot_print_at(double theta);
	cart_point rotate(cart_point point, double theta);
};

#endif