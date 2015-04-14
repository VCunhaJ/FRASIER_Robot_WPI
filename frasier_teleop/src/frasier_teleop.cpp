#include <frasier_teleop/frasier_teleop_main.h>

TeleopFrasier::TeleopFrasier()
{

power_pub = n.advertise<roboteq_mc_nxtgen_driver::RPM>("cmd_power", 1);
velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

//Set 
twist_msg.linear.x = 0;
twist_msg.linear.y = 0;
twist_msg.linear.z = 0;
twist_msg.angular.x = 0;
twist_msg.angular.y = 0;
twist_msg.angular.z = 0;

joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopFrasier::joyCallBack, this);
rpm_msg.data = std::vector<float>(2);
std::string sub_topic = std::string();
UCH = new UI_Command_Handler(n, sub_topic);

};

float TeleopFrasier::Scale(float in, float max_in, float max_out)
{
	return(in/max_in)*max_out;


}

float TeleopFrasier::WithinRange(float in, float max)
{
	if(in > max){
		in = max;
	}
	else if (in < -1*max){
		in = -1*max;

	}
	return in;
}

void TeleopFrasier::PublishMessages()
{
	if(!(UCH->get_flag()))
	{
		return;
	}

	power_pub.publish(rpm_msg);
	velocity_pub.publish(twist_msg);

	ROS_DEBUG("%f, %f", rpm_msg.data[0], rpm_msg.data[1]);
	ROS_DEBUG("%f, %f", twist_msg.linear.x, twist_msg.angular.z);

}


void TeleopFrasier::joyCallBack(const sensor_msgs::Joy::ConstPtr& joyPtr)
{
	rpm_msg.data[1] = WithinRange(speed_to_rpm *(joyPtr->axes[drive_axes] + joyPtr->axes[turn_axes]), max_power);
	rpm_msg.data[0] = WithinRange(speed_to_rpm *-1*(joyPtr->axes[drive_axes] - joyPtr->axes[turn_axes]), max_power);

	twist_msg.linear.x = Scale(joyPtr->axes[drive_axes], max_joy, max_speed);
	twist_msg.angular.z = Scale(joyPtr->axes[turn_axes], max_joy, max_turn);

	PublishMessages();

}



int main (int argc, char** argv)
{
	ros::init(argc, argv, "frasier_teleop_node");

	TeleopFrasier frasier_teleop = TeleopFrasier();

	Rate rate(10);
	while(ok())
	{
		frasier_teleop.PublishMessages();
		spinOnce();
		rate.sleep();

	}
}