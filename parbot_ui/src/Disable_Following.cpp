#include "Command_Pub_Handler.h"

#define PUB_T "UI/Following"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "disable_following");
    ros::NodeHandle n;
    std::string pub_topic = std::string(PUB_T);
    parbot_msgs::Command_MSG cmd_msg;
    cmd_msg.command = "Disable_User_Following";
    cmd_msg.emergency = false;
    cmd_msg.flag = false;
    Command_Pub_Handler* CPH = new Command_Pub_Handler(n, pub_topic, cmd_msg);
}
