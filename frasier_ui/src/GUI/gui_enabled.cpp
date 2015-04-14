#include <frasier_ui/UI_Command_Handler.h>
#include <frasier_ui/FRASIER_Pub_Handler.h>

#define PUB_T "FRASIER_UI/GUI"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "enable_gui_node");
	NodeHandle n;
	frasier_msgs::Command_MSG cmd_msg;
	std::string pub_topic = std::string(PUB_T);
	cmd_msg.command = "Disable_GUI";
	cmd_msg.flag = true;
	cmd_msg.emergency = false;
	FRASIER_Pub_Handler* CPH = new FRASIER_Pub_Handler(n, pub_topic, cmd_msg);