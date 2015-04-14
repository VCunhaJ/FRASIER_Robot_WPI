#include "Pose_TF.h"
#include "nav_msgs/Path.h"

class Path_TF
{
public:
    Path_TF(std::string _target_frame);
    void setPath(nav_msgs::Path old_path);
    nav_msgs::Path TF_Path();
    nav_msgs::Path TF_Path(nav_msgs::Path old_path);
private:
    Pose_TF* pose_tf;
    std::string target_frame;
    nav_msgs::Path original_path;
};
