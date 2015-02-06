/*
 * outlier_segmentation.h
 *
 *  Created on: Dec 14, 2014
 *      Author: Vanderlei Cunha Jr.
 */

#ifndef OUTLIER_SEGMENTATION_H_
#define OUTLIER_SEGMENTATION_H_

#include<Fixtures/virtual_fixtures.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

using namespace std;
using namespace pcl;
using namespace ros;

/*Initialize Publisher and Subscriber*/
Publisher pub_seg;
Subscriber sub_raw_data;

/*Objects for storing manipulated cloud data  */
PointCloud<PointXYZRGB>::Ptr INLIERPoints(new PointCloud<PointXYZRGB> ); // object for storing inlier point clouds
PointCloud<PointXYZRGB>::Ptr segCloudPtr(new PointCloud<PointXYZRGB> ()); // object for storing desired points clouds
PointCloud<PointXYZRGB>::Ptr planeExtractedPtr(new pcl::PointCloud<PointXYZRGB> ());  //object for storing extracted clouds
PointCloud<PointXYZRGB>::Ptr concaveHull(new PointCloud<PointXYZRGB>); // object for displaying concave hull pointcloud
PointCloud<PointXYZRGB>::Ptr planePtr(new PointCloud<PointXYZRGB> ());
PointCloud<PointXYZRGB>::Ptr needsFilterPtr(new PointCloud<PointXYZRGB> ());



/*Segmentation and algorith driven objects*/
ModelCoefficients::Ptr coefficients(new ModelCoefficients); //object for storing model coefficients
SACSegmentation<PointXYZRGB> segmentation; // segmentation object
PointIndices::Ptr inlierIndices(new PointIndices); //point indices object

/*RANSAC Model Fitting*/
PointCloud<PointXYZRGB>::Ptr bigPlanePtr(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr inlierPointsPtr(new PointCloud<PointXYZRGB>);
std::vector<int> inlierIndicesVector;

/*Point Distance Threshold and Max Iteratoin*/
double dist_threshold = 0.15;
int iteration_num = 20;
/*Parameter for filtering point cloud noise*/
double noise_eliminate;
/* ShutDown Request */
bool ransac_node_shutdown = false;

//-------------------------------------------------------------------------------------------------
/*Sensor Message*/
sensor_msgs::PointCloud2 pc_data;

/*Segmentation distance between each segmentation threshold and max iteration*/
double max_thresh = 0.023; //23mm
double max_iteration = 20;
#endif /* OUTLIER_SEGMENTATION_H_ */
