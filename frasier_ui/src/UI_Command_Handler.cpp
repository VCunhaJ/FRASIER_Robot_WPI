#include <frasier_ui/UI_Command_Handler.h>

UI_Command_Handler::UI_Command_Handler(ros::NodeHandle n, std::string sub_topic)
{
	ui_sub = n.subscribe(sub_topic, 1, &UI_Command_Handler::uiCallBack, this);
	got_message = false;
}

void UI_Command_Handler::uiCallBack(const frasier_msgs::Command_MSG cmd_msg)
{
	got_message = true;
	last_msg.command = cmd_msg.command;
	last_msg.ID = cmd_msg.ID;
	last_msg.flag = cmd_msg.flag;
	last_msg.value = cmd_msg.value;
	last_msg.emergency = cmd_msg.emergency;

}

bool UI_Command_Handler::ready()
{
	return got_message;
}

int64_t UI_Command_Handler::get_ID()
{
	return last_msg.ID;

}

bool UI_Command_Handler::get_flag()
{
	return last_msg.flag;

}


double UI_Command_Handler::get_value()
{
	return last_msg.value;

}

bool UI_Command_Handler::get_emergency()
{
	return last_msg.emergency;

}

std::string UI_Command_Handler::get_command()
{
	return last_msg.command;
}
