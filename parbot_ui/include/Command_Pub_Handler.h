#ifndef CMD_PUB_H
#define CMD_PUB_H

#include "ros/ros.h"
#include "parbot_msgs/Command_MSG.h"

#define NUM_PUB 5
#define RATE 10

class Command_Pub_Handler
{
public:
    Command_Pub_Handler(ros::NodeHandle n, std::string pub_topic);
    Command_Pub_Handler(ros::NodeHandle n, std::string pub_topic, parbot_msgs::Command_MSG cmd_msg);
    void publish(parbot_msgs::Command_MSG cmd_msg, int num_to_pub=NUM_PUB);
private:
    ros::Publisher cmd_pub;

    void init(ros::NodeHandle n, std::string pub_topic);
};

#endif
