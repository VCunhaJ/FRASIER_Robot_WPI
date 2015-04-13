#ifndef UI_CMD_H
#define UI_CMD_H

#include "ros/ros.h"
#include "parbot_msgs/Command_MSG.h"

class UI_Command_handler
{
public:
    UI_Command_handler(ros::NodeHandle n, std::string sub_topic);

    bool ready();
    std::string get_command();
    int64_t  get_ID();
    bool get_flag();
    double get_value();
    bool get_emergency();

private:
    ros::Subscriber ui_sub;
    bool got_message;
    parbot_msgs::Command_MSG last_msg;

    void ui_callback(const parbot_msgs::Command_MSG cmd_msg);
};

#endif
