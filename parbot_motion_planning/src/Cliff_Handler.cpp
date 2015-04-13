/* author Kevin Burns */
#include "Cliff_Handler.h"


Cliff_Handler::Cliff_Handler(ros::NodeHandle n, std::string sub_topic)
{
    cliff_sub = n.subscribe(sub_topic, 1, &Cliff_Handler::cliffback, this);
};

void Cliff_Handler::cliffback(const cliff_detection::cliff_status cliff_msg)
{
    last_msg.cliff_status = cliff_msg.cliff_status;
}

int Cliff_Handler::get_status()
{
    return last_msg.cliff_status;
}
