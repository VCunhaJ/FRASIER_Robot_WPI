/*
 * concave_hull.h
 *
 *  Created on: Dec 10, 2014
 *      Author: Vanderlei Cunha Jr.
 */

#ifndef CONCAVE_HULL_H_
#define CONCAVE_HULL_H_

#include<Fixtures/virtual_fixtures.h>

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
using namespace ros;
using namespace pcl;

Subscriber subscribe_object;
Publisher publish_concave;
sensor_msgs::PointCloud2 concaveProjectedCloud;

PointCloud<PointXYZRGB>::Ptr concavePtr (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr concaveCloudPtr (new PointCloud<PointXYZRGB>);
ConcaveHull<PointXYZRGB> concaveObject;


#endif /* CONCAVE_HULL_H_ */
