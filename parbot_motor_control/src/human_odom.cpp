//ROS Includes
#include "ros/ros.h" //ros
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

//Publisher
ros::Publisher twist_pub;

void odomCallback(const nav_msgs::Odometry data){
	geometry_msgs::Twist output; // new twist with euler angles to publish

	output.linear.x = data.pose.pose.position.x;
	output.linear.y = data.pose.pose.position.y;
	output.linear.z = data.pose.pose.position.z	;

	tf::Quaternion q(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w); //define a quaternian
	//q = data.pose.pose.orientation; //save quaternian data from odom message
	/*
	q.x = data.pose.pose.orientation.x;
	q.y = data.pose.pose.orientation.y;
	q.z = data.pose.pose.orientation.z;
	q.w = data.pose.pose.orientation.w;
	*/
	tf::Matrix3x3 m(q);
	double x, y, z;
	m.getRPY(x, y, z); //set x y and z as roll, pitch and yaw.

	//set output twist angular data as orientation from quaternian
	output.angular.x = x;
	output.angular.y = y;
	output.angular.z = z;

	twist_pub.publish(output);
}

int main(int argc, char **argv)
{
	//ROS setup
	ros::init(argc, argv, "Odom_to_Twist");
    ros::NodeHandle n;
    twist_pub = n.advertise<geometry_msgs::Twist>("human_odom", 1);
    //subscriber
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
    ros::Rate loop_rate(20); //try to publish at 1 Hz
    while(ros::ok())
    {
    	ros::spinOnce();
    	loop_rate.sleep();
    }
    return 0;
}