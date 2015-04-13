#include "publisher_handler.h"

#define SUB_T "UI/Following"
#define CLIFF_SUB "cliff_status"

#define SLOW_RATIO 0.25

puiblisher_handler::puiblisher_handler(ros::NodeHandle n, std::string publisher_topic)
{
	vel_pub = n.advertise<geometry_msgs::Twist>(publisher_topic, 1);
	msg = geometry_msgs::Twist();
	msg.linear.x = 0;
	msg.angular.z = 0;
	last_msg = ros::Time::now();
	time_delay = (double)1/(double)PUB_HZ;
    std::string cmd_topic = std::string(SUB_T);
    UCH = new UI_Command_handler(n, cmd_topic);
    std::string cliff_topic = std::string(CLIFF_SUB);
    CLH = new Cliff_Handler(n, cliff_topic);
;};

void puiblisher_handler::publish(tentacle* tent)
{
	publish(tent->get_v(), tent->get_w());
	//tent->print_path();
}

double puiblisher_handler::time_since_last_msg()
{
	ros::Time now = ros::Time::now();
	return ((double)now.sec-(double)last_msg.sec)+((double)now.nsec-(double)last_msg.nsec)/1000000000;
}

bool puiblisher_handler::too_soon()
{
	return time_delay > time_since_last_msg();
}

void puiblisher_handler::publish(double v, double w)
{
    if(!(UCH->get_flag()))
    {
        //ROS_INFO("Following Disabled");
        return;
    }
    else if(UCH->get_emergency())
        return;

    switch (CLH->get_status())
    {
        case 0:
            break;
        case 1:
            v = v*SLOW_RATIO;
            w = w*SLOW_RATIO;
            break;
        case 2:
            v = 0;
            w = 0;
            break;
        case 3:
            v = v*SLOW_RATIO;
            w = w*SLOW_RATIO;
            break;
        default:
            break;
    }

	if(msg.linear.x == v && msg.angular.z == w && too_soon())
	{
		//ROS_INFO("skipping publish %f == %f, %f == %f, %d", msg.linear.x, v, msg.angular.z, w, too_soon());
		return;
	}
	//ROS_INFO("not skipping publish %f == %f, %f == %f, %f < %f", msg.linear.x, v, msg.angular.z, w, time_delay, time_since_last_msg());
	msg.linear.x = v;//0;//
	msg.angular.z = w;//0;//
	if(CAN_MOVE)
	{
		last_msg = ros::Time::now();
		vel_pub.publish(msg);
	}
	//ROS_INFO("Publishing disabled would have sent v=%f w=%f",v,w);
}
