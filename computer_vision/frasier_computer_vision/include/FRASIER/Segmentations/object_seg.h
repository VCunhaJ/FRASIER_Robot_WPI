#ifndef OBJECT_SEG_H_
#define OBJECT_SEG_H_

#include <FRASIER/frasier_main.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <Fixtures/Filters/radius_conditional_filter.h>
using namespace std;
using namespace ros;
using namespace pcl;

/*Publisher and Subscriber*/
Subscriber subscribe_clusters;
Publisher publish_number_of_objects;


/*Objects*/
PointCloud<PointXYZRGB>::Ptr new_cloud(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr clusterObject	(new PointCloud<PointXYZRGB>);


EuclideanClusterExtraction<PointXYZRGB> cluster;
vector<PointIndices> clusterIndices;
search::KdTree<PointXYZRGB>::Ptr kdTree(new search::KdTree<PointXYZRGB>);

sensor_msgs::PointCloud2 objectSegmented;
vector<PointIndices>::const_iterator i;
vector<int>::const_iterator point;


/*Parameters*/
const double CLUSTER_TOLERANCE = 0.005; //(cm)
int currentCluster = 0;

#define MAXIMUM_CLUSTER_SIZE 100
#define MINIMUM_CLUSTER_SIZE 2500





#endif
