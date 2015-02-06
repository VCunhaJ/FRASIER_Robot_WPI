/*
 * fixed_base_tool.h
 *
 *  Created on: Dec 12, 2014
 *      Author: Vanderlei Cunha Jr.
 *
 *
 */

#ifndef FIXED_BASE_TOOL_H_
#define FIXED_BASE_TOOL_H_

#include <Fixtures/virtual_fixtures.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
using namespace std;
using namespace ros;
using namespace pcl;

sensor_msgs::PointCloud2 baseToolPoint;
sensor_msgs::PointCloud2 toolPoint;
PointCloud<PointXYZ>::Ptr baseTool(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr transf_base_tool (new pcl::PointCloud<PointXYZ> ());

PointCloud<PointXYZ>::Ptr base(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr tool(new PointCloud<PointXYZ>);

ros::Publisher publish_baseTipLocation;


//pcl::PointCloud<pcl::PointXYZ>::Ptr base;
//pcl::PointCloud<pcl::PointXYZ>::Ptr tool;



#endif /* FIXED_BASE_TOOL_H_ */
