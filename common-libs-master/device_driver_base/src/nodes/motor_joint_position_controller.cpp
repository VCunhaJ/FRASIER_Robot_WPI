/**
 * @file   motor_joint_position_controller.cpp
 *
 * @date   Mar 26, 2013
 * @author Mitchell Wills
 * @brief  A node which translates between angles of an joint and a motor's raw encoder position 
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
//************ LOCAL DEPENDANCIES ****************//
#include "device_driver_base/SetJointPositionAction.h"
#include "device_driver_base/SetMotorPositionAction.h"
#include "device_driver_base/MotorFeedback.h"
#include "device_driver_base/driver_util.h"
//***********    NAMESPACES     ****************//


class MotorJointPositionController{
	typedef actionlib::SimpleActionServer<device_driver_base::SetJointPositionAction> Server;
	typedef actionlib::SimpleActionClient<device_driver_base::SetMotorPositionAction> Client;

private:
	ros::Publisher joint_pub_;
	ros::Subscriber feedback_sub_;

	boost::shared_ptr<Server> joint_control_server_;
	boost::shared_ptr<Client> motor_control_client_;

	double last_position_;

	/*
	 * Parameters
	 */
	double ticks_per_radian_;
	int32_t zero_tick_position_;
        bool invert_angle_;

	std::string src_frame_;
	std::string joint_name_;

	std::string joint_control_name_;
	std::string motor_control_name_;
	std::string motor_feedback_topic_;


	bool is_angle_bounded_;
	double min_angle_;
	double max_angle_;

	double velocity_zero_threshold_;


private:
	double motor_position_to_angle(int32_t position){
	  if(invert_angle_)
		return -(position-zero_tick_position_)/ticks_per_radian_;
	  else
		return (position-zero_tick_position_)/ticks_per_radian_;
	}
	int32_t angle_to_motor_position(double angle){
	  if(invert_angle_)
		return -(int32_t)(angle * ticks_per_radian_ + zero_tick_position_);
	  else
		return (int32_t)(angle * ticks_per_radian_ + zero_tick_position_);
	}
	float velocity_to_motor_rate(double velocity){
		return (float)(velocity * ticks_per_radian_);
	}

public:
	void motorActionActiveCallback(){
	}
	void motorActionDoneCallback(const actionlib::SimpleClientGoalState &state, const device_driver_base::SetMotorPositionResultConstPtr &result){
		ROS_DEBUG("Joint reached goal %f", motor_position_to_angle(result->final_position));
		device_driver_base::SetJointPositionResult joint_result;
		joint_result.final_angle = motor_position_to_angle(result->final_position);

		if(state == actionlib::SimpleClientGoalState::PREEMPTED){
			if(joint_control_server_->isActive())
				joint_control_server_->setPreempted(joint_result, state.getText());
		}
		else if(state == actionlib::SimpleClientGoalState::ABORTED){
			if(joint_control_server_->isActive())
				joint_control_server_->setAborted(joint_result, state.getText());
		}
		else if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
			if(joint_control_server_->isActive())
				joint_control_server_->setSucceeded(joint_result, state.getText());
		}
		else
			ROS_WARN_STREAM("Got other motor action done state: " << state.toString());
	}
	void motorActionFeedbackCallback(const device_driver_base::SetMotorPositionFeedbackConstPtr &motor_feedback){
		processFeedback(motor_feedback->feedback);
		device_driver_base::SetJointPositionFeedback joint_feedback;
		joint_feedback.header = motor_feedback->feedback.header;
		joint_feedback.current_angle = motor_position_to_angle(motor_feedback->feedback.position);
		joint_control_server_->publishFeedback(joint_feedback);
	}

	void jointActionPreemptCallback(){
		device_driver_base::SetJointPositionResult joint_result;
		joint_result.final_angle = last_position_;
		motor_control_client_->cancelGoal();
	}
	void jointActionGoalCallback(){
		const device_driver_base::SetJointPositionGoalConstPtr goal = joint_control_server_->acceptNewGoal();
		if(goal->max_velocity < velocity_zero_threshold_){
			ROS_ERROR("Got request to go to angle with a max velocity of basically 0 (%f)", goal->max_velocity);
			device_driver_base::SetJointPositionResult joint_result;
			joint_result.final_angle = last_position_;
			joint_control_server_->setAborted(joint_result, "Got request to go to angle with a max velocity of basically 0");
		}
		double angle = goal->angle;
		if(is_angle_bounded_){
			if(angle<min_angle_){
				ROS_WARN("Requested angle (%f) was less than min angle (%f). Limiting to min angle", angle, min_angle_);
				angle = min_angle_;
			}
			if(angle>max_angle_){
				ROS_WARN("Requested angle (%f) was greater than max angle (%f). Limiting to max angle", angle, max_angle_);
				angle = max_angle_;
			}
		}

		ROS_DEBUG("Joint going to angle %f at max velocity %f", goal->angle, goal->max_velocity);
		device_driver_base::SetMotorPositionGoal motor_goal;
		motor_goal.position = angle_to_motor_position(angle);
		motor_goal.max_velocity = velocity_to_motor_rate(goal->max_velocity);
		motor_control_client_->sendGoal(motor_goal,
				boost::bind(&MotorJointPositionController::motorActionDoneCallback, this, _1, _2),
				boost::bind(&MotorJointPositionController::motorActionActiveCallback, this),
				boost::bind(&MotorJointPositionController::motorActionFeedbackCallback, this, _1));
	}


	void motorFeedbackCallback(const device_driver_base::MotorFeedback::ConstPtr& msg) {
		processFeedback(*msg);
	}

	void processFeedback(const device_driver_base::MotorFeedback& feedback){
		last_position_ = motor_position_to_angle(feedback.position);
		sensor_msgs::JointState joint_state;
		joint_state.header.stamp = feedback.header.stamp;
		joint_state.header.seq = feedback.header.seq;
		joint_state.header.frame_id = src_frame_;
		joint_state.name.resize(1);
		joint_state.position.resize(1);
		joint_state.name[0] = joint_name_;
		joint_state.position[0] = last_position_;
		joint_pub_.publish(joint_state);
	}

	MotorJointPositionController(ros::NodeHandle& nh,
                        double ticks_per_radian, int32_t zero_tick_position, bool invert_angle,
			std::string& src_frame, std::string& joint_name,
			std::string& joint_control_name, std::string& motor_control_name,
			std::string& motor_feedback_topic,
			bool is_angle_bounded, double min_angle, double max_angle,
			double velocity_zero_threshold) :
	                ticks_per_radian_(ticks_per_radian), zero_tick_position_(zero_tick_position), invert_angle_(invert_angle),
			src_frame_(src_frame), joint_name_(joint_name),
			joint_control_name_(joint_control_name), motor_control_name_(motor_control_name),
			motor_feedback_topic_(motor_feedback_topic),
			is_angle_bounded_(is_angle_bounded), min_angle_(min_angle), max_angle_(max_angle),
			velocity_zero_threshold_(velocity_zero_threshold){
		joint_control_server_ = boost::shared_ptr<Server>(new Server(nh, joint_control_name_, false));
		joint_control_server_->registerGoalCallback(boost::bind(&MotorJointPositionController::jointActionGoalCallback, this));
		joint_control_server_->registerPreemptCallback(boost::bind(&MotorJointPositionController::jointActionPreemptCallback, this));
		motor_control_client_ = boost::shared_ptr<Client>(new Client(nh, motor_control_name_, false));
		feedback_sub_ = nh.subscribe(motor_feedback_topic_, 1000, &MotorJointPositionController::motorFeedbackCallback, this);
		joint_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
		joint_control_server_->start();
	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_joint_position_controller");
	ros::NodeHandle nh;

	define_and_get_param(double, ticks_per_radian, "~ticks_per_radian", 1);
	define_and_get_param(int32_t, zero_tick_position, "~zero_tick_position", 0);
	define_and_get_param(bool, invert_angle, "~invert_angle", false);

	define_and_get_param(std::string, src_frame, "~src_frame", "motor_base");
	define_and_get_param(std::string, joint_name, "~joint_name", "motor_joint");

	define_and_get_param(std::string, joint_control_name, "~joint_control_name", "joint_control");
	define_and_get_param(std::string, motor_control_name, "~motor_control_name", "motor_control");
	define_and_get_param(std::string, motor_feedback_topic, "~motor_feedback_topic", "motor_feedback");

	define_and_get_param(bool, is_angle_bounded, "~is_angle_bounded", false);
	define_and_get_param(double, min_angle, "~min_angle", -M_PI);
	define_and_get_param(double, max_angle, "~max_angle", M_PI);

	define_and_get_param(double, velocity_zero_threshold, "~velocity_zero_threshold", 0.001);

	MotorJointPositionController position_controller(nh,
			ticks_per_radian, zero_tick_position, invert_angle,
			src_frame, joint_name,
			joint_control_name, motor_control_name,
			motor_feedback_topic,
			is_angle_bounded, min_angle, max_angle,
			velocity_zero_threshold);

	ros::spin();
}
