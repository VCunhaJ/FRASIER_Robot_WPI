#include "Path_TF.h"

Path_TF::Path_TF(std::string _target_frame)
{
    target_frame = _target_frame;
    pose_tf = new Pose_TF(target_frame);
};

nav_msgs::Path Path_TF::TF_Path(nav_msgs::Path old_path)
{
    nav_msgs::Path new_path;
    new_path.header = old_path.header;
    new_path.header.frame_id = target_frame;
    new_path.poses =
    std::vector<geometry_msgs::PoseStamped>(old_path.poses.size
        ());
    for(int i=0; i<old_path.poses.size(); i++)
    {
        new_path.poses[i] = pose_tf->TF_Pose(old_path.poses[i]);
    }
}

void Path_TF::setPath(nav_msgs::Path old_path)
{
    original_path = old_path;
}

nav_msgs::Path Path_TF::TF_Path()
{
    return TF_Path(original_path);
}
