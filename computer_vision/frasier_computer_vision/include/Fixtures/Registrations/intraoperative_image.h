/*
 * intraoperative_image.h
 *
 *  Created on: Dec 16, 2014
 *      Author: Vanderlei Cunha Jr.
 */


#ifndef INTRAOPERATIVE_IMAGE_H_
#define INTRAOPERATIVE_IMAGE_H_

#include<Fixtures/virtual_fixtures.h>

#include<Fixtures/virtual_fixtures.h>
#include<Fixtures/Images/fixed_base_tool.h>


using namespace std;
using namespace ros;
using namespace pcl;

PointCloud<PointXYZRGB> SegmentedObject;
PointCloud<PointXYZRGB>::Ptr SegmentedObjectDisplay (new PointCloud<PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr INLIERPoints(new pcl::PointCloud<pcl::PointXYZRGB> ); // object for storing inlier point clouds
pcl::PointCloud<pcl::PointXYZRGB>::Ptr segCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB> ()); // object for storing desired points clouds
pcl::PointCloud<pcl::PointXYZRGB>::Ptr segCloudPtrFiltered(new pcl::PointCloud<pcl::PointXYZRGB> ()); // object for storing desired points clouds
pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeExtractedPtr(new pcl::PointCloud<pcl::PointXYZRGB> ());  //object for storing extracted clouds
pcl::PointCloud<pcl::PointXYZRGB>::Ptr concaveHull(new pcl::PointCloud<pcl::PointXYZRGB>); // object for displaying concave hull pointcloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePtr(new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr needsFilterPtr(new pcl::PointCloud<pcl::PointXYZRGB> ());
/*Segmentation and algorith driven objects*/
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //object for storing model coefficients
pcl::SACSegmentation<pcl::PointXYZRGB> segmentation; // segmentation object
pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices); //point indices object

ros::Subscriber subscribeSegmentedObject;
ros::Publisher publishPreoperativeImage;


double max_thresh = 0.020; //
double max_iteration = 50;

#endif /* INTRAOPERATIVE_IMAGE_H_ */
