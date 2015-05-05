/* Author: Kevin Burns */
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped"



int main(int argc, char **argv)
{
    //Start ROS
    ros::init(argc, argv, "PARbot_nav_goal");
    ros::NodeHandle n;
	ros::Publisher parbotTarget_pub = n.advertise<geometry_msgs::PoseStamped>("Tracking/object_position", 1);	
}
	
