#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"

class Pose_TF{
public:
    Pose_TF(std::string _target_frame);
    geometry_msgs::PoseStamped TF_Pose(geometry_msgs::PoseStamped old_pose);
private:
    std::string target_frame;
    tf::TransformListener transformer;
};
