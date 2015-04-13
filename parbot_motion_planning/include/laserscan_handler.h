#ifndef LASERSCAN_HANDLER_H
#define LASERSCAN_HANDLER_H

#include <vector>
#include "motion_planning_structures.h"
#include "motion_planning_defs.h"
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

class laserscan_handler{
public:
	laserscan_handler(ros::NodeHandle _n, std::string topic_name=SCAN_TOPIC);

	bool message_received;
	std::vector<grid_cart_point> scan_points;
	ros::NodeHandle n;
	ros::Subscriber laser_sub;

	void callback(const sensor_msgs::LaserScanConstPtr& laser_scan);
	std::vector<grid_cart_point> get_scan_points();
	bool scan_ready();
	std::vector<polar_point> checkForRange(std::vector<polar_point> points, 
		double min_r, double max_r);
private:
    cart_point fake_tf(cart_point point, double x_offset=FAKE_TF_X,
        double y_offset=FAKE_TF_Y);
    std::vector<cart_point> fake_tf(
        std::vector<cart_point> points, double x_offset=FAKE_TF_X,
        double y_offset=FAKE_TF_Y);
};

#endif
