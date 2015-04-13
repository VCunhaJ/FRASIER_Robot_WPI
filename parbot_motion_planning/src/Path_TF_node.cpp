#include "Path_TF.h"
#include "motion_planning_defs.h"

#define PATH_RATE 10
#define SUB_T "parbotPath"
#define PUB_T PATH_TOPIC
#define TARGET_FRAME "/base_link"

class Path_TF_node
{
public:
    Path_TF_node(ros::NodeHandle n, std::string target_frame, std::string pub_topic, std::string sub_topic)
    {
        PTF = new Path_TF(target_frame);
        path_pub = n.advertise<nav_msgs::Path>(pub_topic, 1);
        path_sub = n.subscribe(sub_topic, 1, &Path_TF_node::path_callback, this);
        got_path = false;
    };

    void run()
    {
        ros::Rate loop_rate(PATH_RATE);
        while(ros::ok())
        {
            if(got_path)
                break;
            ros::spinOnce();
            loop_rate.sleep();
        }
        while(ros::ok())
        {
            tfed_path = PTF->TF_Path();
            publish();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }


private:
    Path_TF* PTF;
    ros::Publisher path_pub;
    ros::Subscriber path_sub;
    nav_msgs::Path old_path;
    nav_msgs::Path tfed_path;
    bool got_path;

    void publish()
    {
        path_pub.publish(tfed_path);
    }

    void path_callback(const nav_msgs::PathConstPtr& incoming_path)
    {
        old_path = *incoming_path;
        PTF->setPath(old_path);
        tfed_path = PTF->TF_Path(old_path);
        if(!got_path)
            got_path = true;
        publish();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Path_TF_node");
    ros::NodeHandle n;
    std::string target_frame = std::string(TARGET_FRAME);
    std::string pub_topic = std::string(PUB_T);
    std::string sub_topic = std::string(SUB_T);
    Path_TF_node* ptfn = new Path_TF_node(n, target_frame, pub_topic, sub_topic);
    ptfn->run();
}
