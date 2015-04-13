#include "path_publisher.h"


path_publisher::path_publisher(ros::NodeHandle n)
{
    path_pub = n.advertise<nav_msgs::Path>(PATH_PUB_TOPIC, 1);
    path_msg.header.frame_id = "/base_link";
}

void path_publisher::publish_path(std::vector<cart_point> points)
{
    //ROS_INFO("starting path publish");
    //ROS_INFO("printing %d points", (int)points.size());
    path_msg.poses = Path_From_Points(points);
    path_pub.publish(path_msg);
}

geometry_msgs::PoseStamped path_publisher::get_pose(double _x, double _y, double _theta)
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

//get angle between two points
double path_publisher::getOrientation(cart_point current, cart_point next){
   double x = next.x - current.x;
   double y = next.y - current.y;
   double desired_orientation = atan2(y, x);
   return desired_orientation;
}

//Returns a path message to be published
std::vector<geometry_msgs::PoseStamped> path_publisher::Path_From_Points (std::vector<cart_point> points)
{
   std::vector<geometry_msgs::PoseStamped> _poses = std::vector<geometry_msgs::PoseStamped>(points.size()); //empty vector of poses
   //ROS_INFO("make poses");
   double theta = 0;
   int num_points = points.size();
   //ROS_INFO("staring loop");
   for (int i = 0; i < num_points; ++i)
   {
       cart_point n = points[i];
       if (i < points.size()-1)
       {
           theta = getOrientation(n, points[i+1]);
       }else{
           theta = 0;
       }
       geometry_msgs::PoseStamped pose = get_pose(n.x, n.y, theta);
       //ROS_INFO("About to push pose");
       _poses.push_back(pose);
   }
   return _poses;
}
