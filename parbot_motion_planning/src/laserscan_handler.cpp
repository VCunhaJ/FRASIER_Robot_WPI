#include "laserscan_handler.h"

laserscan_handler::laserscan_handler(ros::NodeHandle _n, std::string topic_name)
{
	message_received = false;
	n = _n;
	laser_sub = n.subscribe(topic_name, 1, &laserscan_handler::callback, this);
	//ROS_INFO("laserscan_handler constructed");
};

std::vector<polar_point> msgs_to_points(std::vector<float> ranges, float min_angle, float angle_increment)
{
	std::vector<polar_point> points;


	return points;
}

std::vector<grid_cart_point> laserscan_handler::get_scan_points()
{
	return scan_points;
}

cart_point laserscan_handler::fake_tf(cart_point point, double x_offset,
 double y_offset)
{
    cart_point new_point;
    new_point.x = point.x + x_offset;
    new_point.y = point.y + y_offset;
    return new_point;
}

std::vector<cart_point> laserscan_handler::fake_tf(
    std::vector<cart_point> points, double x_offset, double y_offset)
{
    for(int i=0; i<points.size(); i++)
    {
        points[i] = fake_tf(points[i]);
    }
    return points;
}

bool laserscan_handler::scan_ready()
{
	return message_received;
}

std::vector<polar_point> laserscan_handler::checkForRange(
	std::vector<polar_point> points, double min_r, double max_r)
{
	int num_good_points = 0;
	std::vector<polar_point> good_points = std::vector<polar_point>(points.size());
	for(int i=0; i<good_points.size(); i++)
	{
		if(points[i].r > max_r || points[i].r < min_r || points[i].r != points[i].r)
		{
			//ROS_INFO("bad polar point r=%f theta=%f", points[i].r, points[i].theta);
			continue;
		}
		else
		{
			good_points[num_good_points] = points[i];
			num_good_points++;
			//ROS_INFO("good polar point r=%f theta=%f", points[i].r, points[i].theta);
		}
	}
	good_points.resize(num_good_points);
	//ROS_INFO("num_good_points=%d", num_good_points);
	return good_points;
}

void laserscan_handler::callback(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
	//ROS_INFO("got a scan message");
	double min_angle, angle_increment, min_r, max_r;
	min_angle = laser_scan->angle_min;
	angle_increment = laser_scan->angle_increment;
	min_r = laser_scan->range_min;
	max_r = laser_scan->range_max;
	std::vector<float> ranges = laser_scan->ranges;
	std::vector<polar_point> polar_points = std::vector<polar_point>(ranges.size());
	int real_p_count = 0;
	for(int i=0; i<polar_points.size(); i++)
	{
		if(ranges[i] > max_r || ranges[i] < min_r)
			continue;
		polar_point p_point;
		p_point.r = ranges[i];
		p_point.theta = laser_scan->angle_max - angle_increment*i;//min_angle + angle_increment*i;
		polar_points[real_p_count] = p_point;
		real_p_count++;
	}
	polar_points.resize(real_p_count);

	scan_points = std::vector<grid_cart_point>(polar_points.size());

	//ROS_INFO("min_r = %f max_r = %f", laser_scan->range_min, laser_scan->range_max);

	polar_points = checkForRange(polar_points, laser_scan->range_min,
		laser_scan->range_max);

	std::vector<cart_point> c_points = polar_to_cart(polar_points);
    c_points = fake_tf(c_points);
    scan_points = cart_to_grid(c_points);
    if(!message_received)
	{
		message_received = true;
		//ROS_INFO("laserscan received");
	}
}
