/* author Kevin Burns */
#ifndef CLIFF_HAND_H
#define CLIFF_HAND_H
#include "cliff_detection/cliff_status.h"
#include "ros/ros.h"

#define DEFAULT_CLIFF_TOP "cliff_status"

class Cliff_Handler
{
public:
    Cliff_Handler(ros::NodeHandle n, std::string sub_topic);

    int get_status();//returns safe until a message says otherwise

private:
    cliff_detection::cliff_status last_msg;
    ros::Subscriber cliff_sub;

    void cliffback(const cliff_detection::cliff_status cliff_msg);
};

#endif
