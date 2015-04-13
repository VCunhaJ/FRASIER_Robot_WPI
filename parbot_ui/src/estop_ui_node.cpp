#include "Command_Pub_Handler.h"

#define PUB_T "emergency_stop"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estop_ui_node");
    ros::NodeHandle n;
    std::string pub_topic = std::string(PUB_T);
    parbot_msgs::Command_MSG cmd_msg;
    cmd_msg.command = "ESTOP";
    cmd_msg.emergency = true;
    cmd_msg.flag = true;
    Command_Pub_Handler* CPH = new Command_Pub_Handler(n, pub_topic, cmd_msg);
}
