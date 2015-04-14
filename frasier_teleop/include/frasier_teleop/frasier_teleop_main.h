#ifndef FRASIER_TELEOP_MAIN_H_ 
#define FRASIER_TELEOP_MAIN_H_ 

#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <roboteq_driver/RoboteqGroupMotorControl.h>
#include <roboteq_mc_nxtgen_driver/RPM.h>
#include <frasier_ui/UI_Command_Handler.h>

using namespace std;
using namespace ros;

#define speed_to_rpm 1
#define drive_axes 1
#define turn_axes 0
#define max_power 0.0
#define max_speed 1.0
#define max_turn 1.0
#define max_joy 1.0

#define CMD_SUB "UI/Teleop"


class TeleopFrasier
{
public:
	/**
	* Constructor
	*/
	TeleopFrasier();
    void PublishMessages();

private:
	NodeHandle n;
	Publisher power_pub;
	Publisher velocity_pub;
	Subscriber joy_sub;
	roboteq_mc_nxtgen_driver::RPM rpm_msg;
	geometry_msgs::Twist twist_msg;
	UI_Command_Handler* UCH; //Handler Pointer object


	void joyCallBack(const sensor_msgs::Joy::ConstPtr& joyPtr);
	/**
	*\brief Callback for the joystick controller
	*@param joyPtr from 
	*/
	float Scale(float in, float max_in, float max_out);

	float WithinRange(float in, float max);

};




#endif
