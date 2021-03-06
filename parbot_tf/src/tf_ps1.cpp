#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "parbot_tf_ps1");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(-0.1, 0.0, -1.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "primesense1_depth_frame", "base_link"));
    /*transform.setOrigin( tf::Vector3(0.0, 0.0, -1.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "primsense2_depth_frame", "base_link"));
    transform.setOrigin( tf::Vector3(0.1, 0.0, -1.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "primsense3_depth_frame", "base_link"));*/
    rate.sleep();
  }
  return 0;
};
