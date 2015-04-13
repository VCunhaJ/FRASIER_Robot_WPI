#ifndef FRASIER_TELEOP_H_ 
#define FRASIER_TELEOP_H_ 

#include <strings>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Twist.h>
#include <roboteq_driver/RoboteqGroupMotorControl.h>
#include <roboteq_mc_nxtgen_driver/RPM.h>

using namespace std;
using namespace ros;

#define speed_to_rpm 1
#define drive_axes 1
#define turn_axes 0
#define max_power 0.0
#define max_speed 1.0
#define max_turn 1.0
#define max_joy 1.0



class TeleopFrasier
{
public:
	/**
	* Constructor
	*/
	TeleopFrasier();

private:
	NodeHandle n;
	Publisher power_pub;
	Publisher velocity_pub;
	Subscriber joy_sub;
	roboteq_mc_nxtgen_driver::RPM rpm_msg;
	geometry_msg::Twist twist_msg;


 




	void joyCallBack(const sensor_msgs::Joy::ConstPtr& joyPtr);

};




#endif