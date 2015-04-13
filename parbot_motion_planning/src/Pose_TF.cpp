#include "Pose_TF.h"

Pose_TF::Pose_TF(std::string _target_frame)
{
    target_frame = _target_frame;
};

geometry_msgs::PoseStamped Pose_TF::TF_Pose(geometry_msgs::PoseStamped old_pose)
{
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
