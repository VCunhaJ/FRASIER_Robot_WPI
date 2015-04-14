#ifndef PATH_PUBLISHER_H
#define PATH_PUBLISHER_H

#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_datatypes.h"
#include "motion_planning_structures.h"

#define PATH_PUB_TOPIC "Active_Path"

class path_publisher{
public:
    path_publisher(ros::NodeHandle n);
    void publish_path(std::vector<cart_point> points);
private:
    nav_msgs::Path path_msg;
    ros::Publisher path_pub;

    double getOrientation(cart_point current, cart_point next);
    geometry_msgs::PoseStamped get_pose(double _x, double _y,
        double _theta);
    std::vector<geometry_msgs::PoseStamped> Path_From_Points (std::vector<cart_point> points);
};

#endif
