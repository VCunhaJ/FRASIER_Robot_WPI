#ifndef EUCLIDEAN_CLUSTER_H_
#define EUCLIDEAN_CLUSTER_H_

#include<FRASIER/frasier_main.h>




#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>

using namespace std;
using namespace ros;
using namespace pcl;

/*Subscriber*/
Subscriber subSegmentedObjects;

/*Object for sorting clusters*/
PointCloud<PointXYZRGB>::Ptr segmentedObjectsPtr (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloudFilteredPtr (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr clustersPtr (new PointCloud<PointXYZRGB>);
search::KdTree<PointXYZRGB>::Ptr kdTreePtr (new search::KdTree<PointXYZRGB>);
vector<PointIndices> clusterIndices;
EuclideanClusterExtraction<PointXYZRGB> euclideanExtraction;

/*Iterators and Serialization*/

std::vector<pcl::PointIndices>::const_iterator it;
std::vector<int>::const_iterator pit;
stringstream ss;
long int clusterDisplay;


/*Thresholds*/
double clusterTolerance = 0.02;
int minSize = 100;
int maxSize = 2500;


#endif
