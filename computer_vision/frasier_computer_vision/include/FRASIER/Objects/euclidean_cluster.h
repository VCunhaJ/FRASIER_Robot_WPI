#include<FRASIER/frasier_main.h>


using namespace std;
using namespace ros;
using namespace pcl;

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
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

/*Iterators*/
int i = 0;
std::vector<pcl::PointIndices>::const_iterator it;
std::vector<int>::const_iterator pit;
stringstream stream;


/*Thresholds*/
double clusterTolerance;
int minSize;
int maxSize;


