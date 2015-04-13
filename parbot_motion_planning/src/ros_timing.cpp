#include "ros/ros.h"
#include <iostream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "timing");
	ros::NodeHandle n;
	ros::Time begin = ros::Time::now();
	/*CODE TO TIME GOES HERE*/
	ros::Time end = ros::Time::now();
	//now to calculate elapsed double
	double elapsed_time = ((double)end.sec-(double)begin.sec)+
	((double)end.nsec-(double)begin.nsec)/1000000000;
	std::cout << elapsed_time << "\n";
	return 0;
}