#ifndef ESTOP_H
#define ESTOP_H

#include "ros/ros.h"
#include "parbot_msgs/Command_MSG.h"
#include "geometry_msgs/Twist.h"

class ESTOP
{
public:
    ESTOP(ros::NodeHandle n, std::string sub_topic, std::string pub_topic);

private:
    ros::Subscriber e_sub;
    ros::Publisher e_pub;
    geometry_msgs::Twist stop_msg;
    bool stop;

    void estop_callback(const parbot_msgs::Command_MSG cmd_msg);
    void publish();
    void vel_callback(const geometry_msgs::Twist vel_msg);
};

#endif
