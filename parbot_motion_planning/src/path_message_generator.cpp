#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_datatypes.h"
#include <vector>
#include <math.h>
#include "motion_planning_structures.h"

//get angle between two points
double getOrientation(cart_point current, cart_point next){
   double x = next.x - current.x;
   double y = next.y - current.y;
   double desired_orientation = atan2(y, x);
   return desired_orientation;
}

//Generates poses
geometry_msgs::PoseStamped get_pose(double _x, double _y, double _theta)
{
   geometry_msgs::PoseStamped poseStamped;

   tf::Quaternion q;
   q = tf::createQuaternionFromRPY(_theta, 0, 0);

   //Set Header
   std::string frame ("/base_link");
   poseStamped.header.frame_id = frame;

   //Set Position
   poseStamped.pose.position.x = _x;
   poseStamped.pose.position.y = _y;
   poseStamped.pose.position.z = 0;

   poseStamped.pose.orientation.x = q.x();
   poseStamped.pose.orientation.y = q.y();
   poseStamped.pose.orientation.z = q.z();
   poseStamped.pose.orientation.w = q.w();

   return poseStamped;
}

//Returns a path message to be published
nav_msgs::Path Path_From_Points (std::vector<cart_point> points){
   vector <geometry_msgs::PoseStamped> _poses; //empty vector of poses
   double theta = 0;
   int num_points = points.size();
   for (int i = 0; i < num_points; ++i)
   {
       cart_point n = points[i];
       if (i < num_points.size()-1)
       {
           theta = getOrientation(n, num_points[i+1]);
       }else{
           theta = 0;
       }
       geometry_msgs::PoseStamped pose = get_pose(n.x, n.y, theta);
       _poses.push_back(pose);
   }
}

