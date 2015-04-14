#include "Pose_TF.h"

#define QR_RATE 10
#define SUB_T "/Tracking/object_position"
#define PUB_T "Tracking/map/object_position"
#define TARGET_F "map"

class QR_Pose_TF{
public:
    QR_Pose_TF(ros::NodeHandle n, std::string target_frame, std::string sub_topic, std::string pub_topic)
    {
        pose_sub = n.subscribe(sub_topic, 1, &QR_Pose_TF::goal_callback, this);
        pose_pub = n.advertise<geometry_msgs::PoseStamped>(pub_topic, 1);
        ros::Rate loop_rate(QR_RATE);
        PTF = new Pose_TF(target_frame);
		ros::spin();
        //got_message = false;
		/*
        while(ros::ok() && !got_message)
		{
            ros::spinOnce();
			loop_rate.sleep();
			//ROS_INFO("wating for message");
		}
        while(ros::ok())
        {
           
            ros::spinOnce();
            loop_rate.sleep();
        }
		*/
    };

private:
    Pose_TF* PTF;
    ros::Publisher pose_pub;
    ros::Subscriber pose_sub;
    geometry_msgs::PoseStamped goal_pose;
    geometry_msgs::PoseStamped output_pose;
    //bool got_message;

    void goal_callback(const geometry_msgs::PoseStampedConstPtr& goal_msg)
    {
		//ROS_INFO("got_message");
        goal_pose = *goal_msg;
        output_pose = PTF->TF_Pose(goal_pose);
        pose_pub.publish(output_pose);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "QR_Pose_TF");
    ros::NodeHandle n;
    std::string target_frame = std::string(TARGET_F);
    std::string sub_topic = std::string(SUB_T);
    std::string pub_topic = std::string(PUB_T);
	//ROS_INFO("calling constructor");
    QR_Pose_TF* QRTF = new QR_Pose_TF(n, target_frame, sub_topic, pub_topic);
	return 0;
}
