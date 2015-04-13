#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "publisher_handler.h"

double speed, pose;

void odom_callback(nav_msgs::OdometryConstPtr& odom_msg)
{
    pose = odom_msg->pose.pose.position.x;
    speed = odom_msg->twist.twist.linear.x;
}

int main(int argc, char **argv)
{
    double start_brake, stopped;
    ros::init(argc, argv, "bracking_test");
    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    //publisher_handler* puh = new publisher_handler(n, "cmd_vel");
    std::string odom = std::string("odom");
    odom_sub = n.subscribe(odom, 1, odom_callback);
    speed = 0;
    pose = 0;
    while(speed < 2 && ros::ok());
    start_brake = pose;
    puh->publish(0,0);
    while(speed != 0 && ros::ok())
        puh->publish(0,0);
    stopped = pose;
    ROS_INFO("bracking distance %f", (stopped-start_brake));
}
