#include <frasier_teleop/frasier_teleop.h>

TeleopFrasier::TeleopFrasier()
{
	

}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "frasier_teleop_node");

	Rate rate(10)
	while(ok())
	{
		spinOnce()
		rate.sleep();

	}
}