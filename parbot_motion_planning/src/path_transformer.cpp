#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "nav_msgs/Path.h"
#include "motion_planning_defs.h"
#include "tf/tf.h"

#define IN_PATH "parbotPath"
#define OUT_PATH PATH_TOPIC
#define TARGET_FRAME "/base_link"//PATH_TARGET_FRAME
#define GOAL_TOPIC "Tracking/map/object_position"

class path_transformer{
public:
		path_transformer(ros::NodeHandle _n, std::string in_path=IN_PATH,
		std::string out_path=OUT_PATH, char* _target_frame=TARGET_FRAME,
		std::string goal_topic=GOAL_TOPIC)
	{
		n = _n;
		path_sub = n.subscribe(in_path, 1, &path_transformer::path_callback,
			this);
		goal_sub = n.subscribe(goal_topic, 1, &path_transformer::goal_callback,
			this);
		path_pub = n.advertise<nav_msgs::Path>(out_path, 1);
		outgoing_msg = nav_msgs::Path();
		ros::Duration tf_dur(1.0);
		target_frame = _target_frame;
		new_path = false;
		new_goal = false;
		ready = false;
        target_frame = std::string(TARGET_FRAME);
	};

	void path_callback(const nav_msgs::PathConstPtr& incoming_path)
	{
		//ROS_INFO("got new path");
		outgoing_msg = *incoming_path;
        //ROS_INFO("starting transform");
		//*
		std::vector<geometry_msgs::PoseStamped> original_poses =
			incoming_path->poses;
		old_poses = std::vector<geometry_msgs::PoseStamped>(original_poses.size());
		for(int i=0; i<original_poses.size(); i++)
		{
			geometry_msgs::PoseStamped new_pose = transform_pose(original_poses[i]);
           	outgoing_msg.poses[i] = new_pose;
			geometry_msgs::PoseStamped old_pose = original_poses[i];
			old_poses[i] = old_pose;
		}
		// */
		//old_poses = incoming_path->poses;	
		outgoing_msg.header.frame_id = target_frame;
		new_path = true;
		publish_path();
	}

	void goal_callback(const geometry_msgs::PoseStampedConstPtr& goal_msg)
	{
		goal_pose = *goal_msg;
		new_goal = true;
		publish_path();
	}

	void update_poses()
	{
		tf_poses();
		//for(int i=0; i<poses.size(); i++)
			//outgoing_msg.poses[i] = poses[i];
	}

	void tf_poses()
	{
		std::vector<geometry_msgs::PoseStamped> poses = std::vector<geometry_msgs::PoseStamped>(old_poses.size());
		for(int i=0; i<old_poses.size(); i++)
		{
			outgoing_msg.poses[i] = transform_pose(old_poses[i]);
		}	
	}

	void publish_path()
	{
		if(!ready)
		{
			ready = new_path;
			if(!ready)
				return;
		}
		//if(new_path)
		//{
		update_poses();
		path_pub.publish(outgoing_msg);
		//}
	}

private:
	ros::NodeHandle n;
	ros::Subscriber path_sub;
	ros::Subscriber goal_sub;
	nav_msgs::Path outgoing_msg;
	ros::Publisher path_pub;
	tf::TransformListener transformer;
	std::vector<geometry_msgs::PoseStamped> old_poses, poses;
	

	std::string target_frame;
	geometry_msgs::PoseStamped goal_pose;
	bool new_path, new_goal, ready;

	geometry_msgs::PoseStamped transform_pose(geometry_msgs::PoseStamped
		old_pose)
	{
		/*
		geometry_msgs::PoseStamped new_pose;
		geometry_msgs::PointStamped new_point;
		geometry_msgs::PointStamped old_point;
		old_point.point = old_pose.pose.position;
		old_point.header = old_pose.header;
		try
		{
			//ros::Time now = ros::Time::now();
			transformer.transformPoint(target_frame,old_point,new_point);
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("TF_ERROR:%s",ex.what());
		}
		//ROS_INFO("source frame %s", old_pose.header.frame_id.c_str()	);
		new_pose = old_pose;
		new_pose.pose.position = new_point.point;
		new_pose.header.frame_id = target_frame;
		//ROS_INFO("point frame = %s", new_pose.header.frame_id.c_str());
		return new_pose;
		*/
		geometry_msgs::PoseStamped new_pose;
	    try
	    {
			//ros::Time now = ros::Time::now();
			transformer.transformPose(target_frame,old_pose,new_pose);
	    }
	    catch (tf::TransformException ex)
	    {
			ROS_INFO("TF_ERROR:%s",ex.what());
	    }
	    return new_pose;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Path_transform");
    ros::NodeHandle n;
    path_transformer* pt = new path_transformer(n);
 	ros::Rate loop_rate(1000);
	while(ros::ok())
	{
		pt->publish_path();
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}
