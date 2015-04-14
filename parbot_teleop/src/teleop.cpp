#include <ros/ros.h>
#include <roboteq_driver/RoboteqGroupMotorControl.h>
#include <sensor_msgs/Joy.h>

#define left_mc 1
#define right_mc 1
#define control_mode 1
#define drive_axes 1
#define turn_axes 0

class Parbot_Teleop
{
public:
  Parbot_Teleop()
  {
    control_msg.motors = std::vector<roboteq_driver::RoboteqMotorControl>(2);
    control_msg.motors[0] = left_msg;
    control_msg.motors[1] = right_msg;
    left_msg.channel = left_mc;
    right_msg.channel = right_mc;
    left_msg.control_mode = control_mode;
    right_msg.control_mode = control_mode;
    left_msg.setpoint = 0;
    right_msg.setpoint = 0;
    driver_pub = nh_.advertise<roboteq_driver::RoboteqGroupMotorControl>("motor_control", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Parbot_Teleop::joyCallback, this);
  };

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher driver_pub;
  ros::Subscriber joy_sub_;
  roboteq_driver::RoboteqGroupMotorControl control_msg;
  roboteq_driver::RoboteqMotorControl left_msg, right_msg;

  void Parbot_Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    left_msg.setpoint = joy->axes[drive_axes] - joy->axes[turn_axes];
    right_msg.setpoint = joy->axes[drive_axes + joy->axes[turn_axes];]\
    driver_pub.publish(control_msg);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Parbot_Teleop");

  Parbot_Teleop parbot_teleop = Parbot_Teleop();

  ros::spin();
}