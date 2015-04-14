#include <ros/ros.h>
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include <math.h>
#include <vector>
#include <algorithm>

class choose_blobs
{

public:

    choose_blobs()
    {
        ch_blobs_pub = n.advertise<cmvision::Blobs>("ch_blobs",1);
        blobs_in = n.subscribe("blobs", 1,
            &choose_blobs::ps1_callback, this);
        color1.r = 83;
        color1.g = 9;
        color1.b = 4;

        color2.r = 255;
        color2.g = 255;
        color2.b = 120;
        possible_targets = std::vector<target_data>(3);
    };



private:

    struct color{
        int r,g,b;
    };

    struct target_data{
        int x,y;
        double score;
        cmvision::Blob blob1,blob2;
        std::string frame_id;
    };

    color color1,color2;
    ros::NodeHandle n;
    std::string output_topic;
    ros::Publisher ch_blobs_pub;
    ros::Subscriber blobs_in;
    cmvision::Blobs out_msg;
    std::vector<target_data> possible_targets;

    double color_diff(color diff_color, cmvision::Blob blob)
    {
        return (double)pow((pow((diff_color.r-blob.red),2)+pow((diff_color.g-blob.red),2)+pow((diff_color.b-blob.blue),2)),0.5);
    }

    int how_many_color1(std::vector<cmvision::Blob> test_blobs)
    {
        int num_blobs = test_blobs.size();
        int match_count = 0;
        for(int i=0; i<num_blobs; i++)
        {
            if(color_diff(color1,test_blobs[i]) < color_diff(color2,test_blobs[i]))
            {
                match_count++;
            }
        }
        return match_count;
    }

    double score_blobs(cmvision::Blob blob1, cmvision::Blob blob2)
    {
        return 1 / pow((pow((blob1.x-blob2.x),2)+pow((blob1.y-blob2.y),2)),0.5);
    }

    void ps1_callback(const cmvision::Blobs::ConstPtr& blobs_msg_ptr)
    {
        //ROS_ERROR("Found something");
        blobs_callback(blobs_msg_ptr, std::string("primesense1_depth_frame"), 0);
    }

    void blobs_callback(const cmvision::Blobs::ConstPtr& blobs_msg_ptr, std::string frame_id, int pos_index)
    {
        //ROS_ERROR("In callback");
        cmvision::Blobs o_blobs_msg = *blobs_msg_ptr;
        std::vector<cmvision::Blob> blobs1,blobs2;
        std::vector<target_data> new_targets;
        int countc1 = 0;
        int countc2 = 0;
        ROS_INFO("Total blobs: %d", o_blobs_msg.blob_count);
        countc1 = how_many_color1(o_blobs_msg.blobs);
        countc2 = o_blobs_msg.blobs.size() - countc1;
        blobs1 = std::vector<cmvision::Blob>(countc1);
        blobs2 = std::vector<cmvision::Blob>(countc2);
        new_targets = std::vector<target_data>(countc1*countc2);
        //ROS_ERROR("LOOKING FOR targets");
        int count = 0;
        for(int i=0; i<blobs1.size(); i++)
        {
            for(int j=0; j<blobs2.size(); i++)
            {
                target_data new_data;
                new_data.blob1 = blobs1[i];
                new_data.blob2 = blobs2[j];
                new_data.x = (new_data.blob1.x + new_data.blob2.x) / 2;
                new_data.y = (new_data.blob1.y + new_data.blob2.y) / 2;
                new_data.frame_id = frame_id;
                new_data.score = score_blobs(new_data.blob1, new_data.blob2);
                new_targets[count] = new_data;
                count++;
            }
        }
        ROS_ERROR("Sorting Targets");
        target_data best_target;
        ROS_ERROR("Named best");
        if(new_targets.size() == 0)
        {
            return;
        }
        best_target.score = new_targets[0].score;
        ROS_ERROR("Generate best");
        for(int i=0; i<new_targets.size(); i++)
        {
            ROS_ERROR("I'm in the loop");
            if(new_targets[i].score > best_target.score)
            {
                best_target = new_targets[i];
            }
        }
        ROS_ERROR("best local found");
        possible_targets[pos_index] = best_target;
        target_data best_target_of_all = best_target;
        for(int i=0; i<possible_targets.size(); i++)
        {
            if(best_target_of_all.score < possible_targets[i].score)
            {
                best_target_of_all = possible_targets[i];
            }
        }
        ROS_ERROR("Preparing to publish");
        std::vector<cmvision::Blob> best_blobs = std::vector<cmvision::Blob>(1);
        cmvision::Blob best_blob;
        best_blob.x = best_target_of_all.x;
        best_blob.y = best_target_of_all.y;
        best_blob.left = std::min(best_target_of_all.blob1.left, best_target_of_all.blob2.left);
        best_blob.right = std::max(best_target_of_all.blob1.right, best_target_of_all.blob2.right);
        best_blob.top = std::min(best_target_of_all.blob1.top, best_target_of_all.blob2.top);
        best_blob.bottom = std::max(best_target_of_all.blob1.bottom, best_target_of_all.blob2.bottom);
        best_blob.area = (best_blob.right - best_blob.left)*(best_blob.bottom - best_blob.top);
        best_blobs[0] = best_blob;


        o_blobs_msg.header.frame_id = best_target_of_all.frame_id;
        o_blobs_msg.blob_count = 1;
        o_blobs_msg.blobs = best_blobs;
        ch_blobs_pub.publish(o_blobs_msg);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parbot_choose_blobs");

    choose_blobs cb = choose_blobs();

    ros::spin();

    return 0;
};
