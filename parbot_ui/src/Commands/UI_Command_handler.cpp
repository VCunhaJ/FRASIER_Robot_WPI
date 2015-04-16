#include "UI_Command_handler.h"

UI_Command_handler::UI_Command_handler(ros::NodeHandle n, std::string sub_topic)
{
    ui_sub = n.subscribe(sub_topic, 1, &UI_Command_handler::ui_callback, this);
    got_message = false;
};

void UI_Command_handler::ui_callback(const parbot_msgs::Command_MSG cmd_msg)
{
    got_message = true;
    last_msg.command = cmd_msg.command;
    last_msg.ID = cmd_msg.ID;
    last_msg.flag = cmd_msg.flag;
    last_msg.value = cmd_msg.value;
    last_msg.emergency = cmd_msg.emergency;
}

bool UI_Command_handler::ready()
{
    return got_message;
}

std::string UI_Command_handler::get_command()
{
    return last_msg.command;
}

int64_t  UI_Command_handler::get_ID()
{
    return last_msg.ID;
}

bool UI_Command_handler::get_flag()
{
    return last_msg.flag;
}

double UI_Command_handler::get_value()
{
    return last_msg.value;
}

bool UI_Command_handler::get_emergency()
{
    return last_msg.emergency;
}


