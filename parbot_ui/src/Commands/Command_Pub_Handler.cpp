#include "Command_Pub_Handler.h"

Command_Pub_Handler::Command_Pub_Handler(ros::NodeHandle n, std::string pub_topic)
{
    init(n, pub_topic);
};

Command_Pub_Handler::Command_Pub_Handler(ros::NodeHandle n, std::string pub_topic, parbot_msgs::Command_MSG cmd_msg)
{
    init(n, pub_topic);
    publish(cmd_msg);
}

void Command_Pub_Handler::init(ros::NodeHandle n, std::string pub_topic)
{
    cmd_pub = n.advertise<parbot_msgs::Command_MSG>(pub_topic, 100);
}

void Command_Pub_Handler::publish(parbot_msgs::Command_MSG cmd_msg, int num_to_pub)
{
    ros::Rate looprate(RATE);
    for(int i=0; i<num_to_pub; i++)
    {
        cmd_pub.publish(cmd_msg);
        ros::spinOnce();
        looprate.sleep();
    }
}
