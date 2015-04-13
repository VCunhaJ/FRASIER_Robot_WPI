#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "publisher_handler.h"

#define STOP_SPEED 1.4

double speed, pose;

void odom_callback(const nav_msgs::Odometry odom_msg)
{
    pose = odom_msg.pose.pose.position.x;
    speed = odom_msg.twist.twist.linear.x;
    ROS_INFO("speed %f", speed);
}

int main(int argc, char **argv)
{
    double start_brake, stopped;
    ros::init(argc, argv, "bracking_test");
    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    puiblisher_handler* puh = new puiblisher_handler(n);
    std::string odom = std::string("odom");
    odom_sub = n.subscribe<nav_msgs::Odometry>(odom,
        1, odom_callback);
    speed = 0;
    pose = 0;
    while(speed < STOP_SPEED && ros::ok())
        ros::spinOnce();
    start_brake = pose;
    puh->publish(0,0);
    while(speed != 0 && ros::ok())
    {
        puh->publish(0,0);
        ros::spinOnce();
    }
    stopped = pose;
    ROS_INFO("bracking distance %f", (stopped-start_brake));
}
