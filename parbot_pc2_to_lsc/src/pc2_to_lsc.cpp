#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointField.h"

// For transforming ROS/OpenCV images
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Messages
#include "sensor_msgs/Image.h"
#include "tf/tfMessage.h"

// Boost
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include "boost/format.hpp"

#include <iostream>
#include <cstdlib>

struct cartPoint {
	float x;
	float y;
	float z;
};


bool zero_point(cartPoint point)
{
	return point.x == point.y && point.x == point.z && point.x == 0;
}

float max(float a, float b)
{
	if(a>=b)
	{
		return a;
	}
	else
	{
		return b;
	}
}

void newPointsCallBack(const sensor_msgs::Image& img_msg)
{

	cv::Mat image;

	//if (img_msg.encoding == "32FC1") { // depth image
	cv_bridge::CvImagePtr depth_image_ptr;
	try {
		depth_image_ptr = cv_bridge::toCvCopy(img_msg);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	//	float z = depth_image_ptr->image.at<float>(0,0);
	//	std::cout << z << '\n';
	//	}
	//	else
	//	{
	//		std::cout << "encoding:" << img_msg.encoding << '\n';
	//	}

	// find max depth
	float max_depth = 0;
	for(int y = 0; y < depth_image_ptr->image.rows; y++) {
		for(int x = 0; x < depth_image_ptr->image.cols; x++) {
			float distance = depth_image_ptr->image.at<int>(y, x);
			if (distance == distance) { // exclude NaN
				max_depth = max(distance, max_depth);
			}
		}
	}

	std::cout << max_depth << '\n';
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl2_to_lsc");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("camera/depth/image_raw", 2, newPointsCallBack);
	ros::spin();
	return 0;
}


