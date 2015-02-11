#ifndef EUCLIDEAN_CLUSTER_H_
#define EUCLIDEAN_CLUSTER_H_

#include<FRASIER/frasier_main.h>


using namespace std;
using namespace ros;
using namespace pcl;

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

/*Subscriber*/
Subscriber subSegmentedObjects;

/*Object for sorting clusters*/
PointCloud<PointXYZ>::Ptr segmentedObjectsPtr (new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr cloudFilteredPtr (new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr clustersPtr (new PointCloud<PointXYZ>);
search::KdTree<PointXYZ>::Ptr kdTreePtr (new search::KdTree<PointXYZ>);
vector<PointIndices> clusterIndices;
EuclideanClusterExtraction<PointXYZ> euclideanExtraction;

/*Iterators and Serialization*/

std::vector<pcl::PointIndices>::const_iterator it;
std::vector<int>::const_iterator pit;
stringstream ss;
int clusterDisplay;


/*Thresholds*/
double clusterTolerance = 0.05;
int minSize = 1000;
int maxSize = 6000;


#endif
