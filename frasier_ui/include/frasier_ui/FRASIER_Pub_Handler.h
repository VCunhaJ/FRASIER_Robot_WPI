#ifndef FRASIER_PUB_HANDLER_H_
#define FRASIER_PUB_HANDLER_H_

#include <ros/ros.h>
#include <frasier_msgs/Command_MSG.h>
#include <frasier_ui/UI_Command_Handler.h>

using namespace std;
using namespace ros;

#define NUM_PUB 5
#define RATE 5

class FRASIER_Pub_Handler
{
public:
	FRASIER_Pub_Handler(ros::NodeHandle n, std::string pub_topic);
	FRASIER_Pub_Handler(ros::NodeHandle n, std::string pub_topic, frasier_msgs::Command_MSG cmd_msg);

	void publishMSG(frasier_msgs::Command_MSG msg, int num_to_pub = NUM_PUB);

private:
	Publisher cmd_pub;

	void initPub(ros::NodeHandle n, std::string pub_topic);


};

#endif