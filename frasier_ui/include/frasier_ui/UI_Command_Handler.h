//This Header File initializes the messages needed for user interface (UI) handler 

#ifndef UI_COMMAND_HANDLER_H_
#define UI_COMMAND_HANDLER_H_

#include <ros/ros.h>
#include <frasier_msgs/Command_MSG.h>

using namespace std;
using namespace ros;


class UI_Command_Handler
{
public:
	UI_Command_Handler(ros::NodeHandle n, std::string sub_topic);
	bool ready();
	int64_t get_ID();
	bool get_flag();
	double get_value();
	bool get_emergency();
	std::string get_command();

private:
	Subscriber ui_sub;
	bool got_message;
	frasier_msgs::Command_MSG last_msg;

	void uiCallBack(const frasier_msgs::Command_MSG cmd_msg);

};



#endif
