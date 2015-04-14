#include <frasier_ui/FRASIER_Pub_Handler.h>

FRASIER_Pub_Handler::FRASIER_Pub_Handler(ros::NodeHandle n, std::string pub_topic)
{
	initPub(n, pub_topic);

}

FRASIER_Pub_Handler::FRASIER_Pub_Handler(ros::NodeHandle n, std::string pub_topic, frasier_msgs::Command_MSG cmd_msg)
{
	initPub(n, pub_topic);
	publishMSG(cmd_msg);

}

void FRASIER_Pub_Handler::initPub(ros::NodeHandle n, std::string pub_topic)
{
		cmd_pub = n.advertise<frasier_msgs::Command_MSG>(pub_topic, 1);

}

void FRASIER_Pub_Handler::publishMSG(frasier_msgs::Command_MSG cmd_msg, int num_to_pub)
{
	Rate loopRate(RATE);
	for(int i = 0; i < num_to_pub; i++)
	{
		cmd_pub.publish(cmd_msg);
		spinOnce();
		loopRate.sleep();
	}

}

