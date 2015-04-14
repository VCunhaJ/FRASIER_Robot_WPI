#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "parbot_sh_test");

	ROS_INFO("Hello");
	
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("hello", 1, callback);

	ros::spin();

	return 0;
};
