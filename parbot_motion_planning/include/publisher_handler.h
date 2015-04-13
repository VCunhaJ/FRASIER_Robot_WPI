#ifndef PUBLISHER_HANDLER_H
#define PUBLISHER_HANDLER_H

#include "motion_planning_structures.h"
#include "tentacle.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "UI_Command_handler.h"
#include "Cliff_Handler.h"

#define CAN_MOVE true//false//
#define PUB_HZ 10.0

class puiblisher_handler{
public:

	puiblisher_handler(ros::NodeHandle n, std::string publisher_topic=PUBLISHER_TOPIC);

	void publish(tentacle* tent);
	void publish(double v, double w);

private:
	ros::Publisher vel_pub;
    UI_Command_handler* UCH;
	geometry_msgs::Twist msg;
	ros::Time last_msg;
	double time_delay;
    Cliff_Handler* CLH;

	double time_since_last_msg();
	bool too_soon();
};

#endif
