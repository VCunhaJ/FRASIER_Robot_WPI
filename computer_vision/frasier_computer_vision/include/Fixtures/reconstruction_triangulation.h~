/*
 * reconstruction_triangulation.h
 *
 *  Created on: Dec 16, 2014
 *      Author: Vanderlei Cunha Jr.
 */

#ifndef RECONSTRUCTION_TRIANGULATION_H_
#define RECONSTRUCTION_TRIANGULATION_H_
#include<Fixtures/virtual_fixtures.h>
using namespace std;
using namespace ros;
using namespace pcl;

ros::Subscriber sub;
ros::Publisher pub;

search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);
PointCloud <PointXYZRGB>::Ptr cloud (new PointCloud <PointXYZRGB>);
sensor_msgs::PointCloud2 pc_data;

IndicesPtr indices (new std::vector <int>);
RegionGrowingRGB<PointXYZRGB> regionGrowing;
std::vector <pcl::PointIndices> clusters;
PassThrough<PointXYZRGB> pass;
#endif /* RECONSTRUCTION_TRIANGULATION_H_ */
