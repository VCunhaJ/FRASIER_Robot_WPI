#include "ESTOP.h"

ESTOP::ESTOP(ros::NodeHandle n, std::string sub_topic, std::string pub_topic)
{
    e_sub = n.subscribe(sub_topic, 1, &ESTOP::estop_callback, this);
    e_pub = n.advertise<geometry_msgs::Twist>(pub_topic, 1);
    stop_msg.linear.x = 0;
    stop_msg.angular.z = 0;
    stop = false;
};

void ESTOP::estop_callback(const parbot_msgs::Command_MSG cmd_msg)
{
    if(cmd_msg.emergency)
    {
        publish();
        stop = true;
    }
    else
        stop = false;
}

void ESTOP::vel_callback(const geometry_msgs::Twist vel_msg)
{
    if(stop)
    {
        if(vel_msg.linear.x != 0 && vel_msg.angular.y != 0)
        {
            publish();
        }
    }
}

void ESTOP::publish()
{
    e_pub.publish(stop_msg);
}
