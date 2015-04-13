#include <ros/ros.h>
#include <roboteq_mc_nxtgen_driver/RPM.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <UI_Command_handler.h>

#define speed_to_rpm 1
#define drive_axes 1
#define turn_axes 0
#define max_power 0.0//10
#define max_speed 1.0//0//1.5// 2.5//
#define max_turn 1.0//1.0//` 0.25//
#define max_joy 1.0

#define CMD_SUB "UI/Teleop"

class Parbot_Teleop
{
public:
  Parbot_Teleop()
  {

    power_pub = nh.advertise<roboteq_mc_nxtgen_driver::RPM>("cmd_power", 1);
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	twist_msg.linear.x = 0;
  twist_msg.linear.y = 0;
	twist_msg.linear.z = 0;
	twist_msg.angular.x = 0;
	twist_msg.angular.y = 0;
	twist_msg.angular.z = 0;

  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Parbot_Teleop::joyCallback, this);
    rpm_msg.data = std::vector<float>(2);

    std::string sub_topic = std::string(CMD_SUB);
    UCH = new UI_Command_handler(nh, sub_topic);
  };

void publish()
{

    if(!(UCH->get_flag()))
    {
        return;
    }

    power_pub.publish(rpm_msg);
	vel_pub.publish(twist_msg);
    ROS_DEBUG("%f, %f", rpm_msg.data[0], rpm_msg.data[1]);
	ROS_DEBUG("%f, %f", twist_msg.linear.x, twist_msg.angular.z);
  }

private:
  ros::NodeHandle nh;
  ros::Publisher power_pub;
  ros::Publisher vel_pub;
  ros::Subscriber joy_sub_;
  roboteq_mc_nxtgen_driver::RPM rpm_msg;
  geometry_msgs::Twist twist_msg;
  UI_Command_handler* UCH;

  float within_range(float in,  float max)
  {
    if(in > max)
    {
      in = max;
    }
    else if(in < -1*max)
    {
      in = -1*max;
    }

    return in;
  }

	float scale(float in, float max_in, float max_out)
	{
		return (in/max_in)*max_out;
	}

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    rpm_msg.data[1] = within_range(speed_to_rpm*(joy->axes[drive_axes] + joy->axes[turn_axes]), max_power);
    rpm_msg.data[0] = within_range(speed_to_rpm*-1*(joy->axes[drive_axes] - joy->axes[turn_axes]), max_power);

	twist_msg.linear.x = scale(joy->axes[drive_axes], max_joy, max_speed);
	twist_msg.angular.z = scale(joy->axes[turn_axes], max_joy,max_turn);
  publish();
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "parbot_teleop_node");

  Parbot_Teleop parbot_teleop = Parbot_Teleop();

  ros::Rate rate(10);

  while(ros::ok())
  {
    parbot_teleop.publish();
    ros::spinOnce();
    rate.sleep();
  }
}
