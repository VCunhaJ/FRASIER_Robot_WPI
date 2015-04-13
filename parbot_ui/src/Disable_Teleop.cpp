#include "Command_Pub_Handler.h"

#define PUB_T "UI/Teleop"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Disable_teleop");
    ros::NodeHandle n;
    std::string pub_topic = std::string(PUB_T);
    parbot_msgs::Command_MSG cmd_msg;
    cmd_msg.command = "Disable_Teleop";
    cmd_msg.emergency = false;
    cmd_msg.flag = false;
    Command_Pub_Handler* CPH = new Command_Pub_Handler(n, pub_topic, cmd_msg);
}
