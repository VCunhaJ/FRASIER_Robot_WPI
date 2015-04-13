#include "robot_footprint.h"

robot_footprint::robot_footprint(double length, double width, double origin_x, double origin_y, 
	double grid_size)
{
	grid_size = GRID_SIZE;
	int num_x = length/grid_size;
	int num_y = width/grid_size;
	cart_point start_point;
	start_point.x = -1*origin_x;
	start_point.y = origin_y-width;
	foot_print = std::vector<cart_point>(num_x*num_y);
	double delta_x = length/(double)num_x;
	double delta_y = width/(double)num_y;
	for(int i=0; i<num_x; i++)
	{
		double current_x = start_point.x + delta_x*i;
		for(int j=0; j<num_y; j++)
		{
			cart_point current_point;
			current_point.x = current_x;
			current_point.y = start_point.y + delta_y*j;
			foot_print[i*num_y+j] = current_point;
		}
	}

	int j = 1;
};

void robot_footprint::print_footprint()
{
	print_cart_points(foot_print);
}

cart_point robot_footprint::rotate(cart_point point, double theta)
{
	cart_point rotated_point;
	rotated_point.x = point.x*cos(theta)-point.y*sin(theta);
	rotated_point.y = point.x*sin(theta)+point.y*cos(theta);
	return rotated_point;
}

std::vector<cart_point> robot_footprint::foot_print_at(double theta)
{
	std::vector<cart_point> rotated_foot_print = std::vector<cart_point>(foot_print.size());
	for(int i=0; i<rotated_foot_print.size(); i++)
	{
		rotated_foot_print[i] = rotate(foot_print[i], theta);
	}
	return rotated_foot_print;
}

void robot_footprint::print_footprint_at(double theta)
{
	print_cart_points(foot_print_at(theta));
}
