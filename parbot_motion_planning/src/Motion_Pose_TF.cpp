#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "nav_msgs/Path.h"
#include "tf/tf.h"

#define TARGET_FRAME "/map" //frame to transform pose to

class pose_transformer{
public:
    pose_transformer(ros::NodeHandle _n)
    {
        n = _n;
        //Publisher
        target_frame = std::string(TARGET_FRAME);
        pose_pub = n.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);

        init_pose.header.frame_id = std::string("base_footprint");
        new_pose.header.frame_id = std::string(TARGET_FRAME);

        init_pose.pose.position.x = 0;
        init_pose.pose.position.y = 0;
        init_pose.pose.position.z = 0;

        init_pose.pose.orientation.x = 0;
        init_pose.pose.orientation.y = 0;
        init_pose.pose.orientation.z = 0;
        init_pose.pose.orientation.w = 1;
    };


    ros::NodeHandle n;
    ros::Publisher pose_pub;
    tf::TransformListener transformer;
    geometry_msgs::PoseStamped output;
    geometry_msgs::PoseStamped init_pose;
    std::string target_frame;

    void transform_pose()
    {
        geometry_msgs::PoseStamped new_pose;
        geometry_msgs::PoseStamped old_pose;

        //Create pose at base_footprint
        try
        {
            transformer.transformPose(target_frame, old_pose, new_pose);
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("TF_ERROR:%s",ex.what());
        }

        pose_pub.publish(new_pose);
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("Started main");
    ros::init(argc, argv, "Pose_transform");
    ros::NodeHandle n;
    ROS_INFO("made node");
    ros::Rate loop_rate(10);
    pose_transformer* pt = new pose_transformer(n); //create pose transformer
    while(ros::ok())
    {
        pt->transform_pose();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0; // should never get here
}
