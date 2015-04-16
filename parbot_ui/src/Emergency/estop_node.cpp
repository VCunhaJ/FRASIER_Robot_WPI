#include "ESTOP.h"

#define SUB_T "emergency_stop"
#define PUB_T "cmd_vel"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "estop_node");
    ros::NodeHandle n;
    std::string pub_topic = std::string(PUB_T);
    std::string sub_topic = std::string(SUB_T);
    ESTOP* estop = new ESTOP(n, sub_topic, pub_topic);
    ros::spin();
}
