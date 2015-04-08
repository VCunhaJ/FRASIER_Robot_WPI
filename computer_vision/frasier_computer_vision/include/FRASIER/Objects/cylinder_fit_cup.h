#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace ros;
using namespace pcl;

//Subscriber and Publisher
Subscriber objectSub;
Publisher cylinderPub;


//PointCloud<PointXYZRGB>::Ptr clustersPtr (new PointCloud<PointXYZRGB>);

// Function Class Object(s) Declared
PassThrough<PointXYZ> pass;
NormalEstimation<PointXYZ, Normal> normalEstimation; //Input: PointXYZ, Output: Normals
SACSegmentationFromNormals<PointXYZ, Normal> seg; 

ExtractIndices<PointXYZ> extract_pts_indices;
ExtractIndices<Normal> extract_normals_indices;
search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());
  
// Futile Class Object(s) Declared
//PointCloud<PointXYZRGB>::Ptr objectPtr (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZ>::Ptr objectPtr (new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr cloud_filteredPtr (new PointCloud<PointXYZ>);
PointCloud<Normal>::Ptr cloud_normalsPtr (new PointCloud<Normal>);
PointCloud<PointXYZ>::Ptr cloud_filtered2Ptr (new PointCloud<PointXYZ>);
PointCloud<Normal>::Ptr cloud_normals2Ptr (new PointCloud<Normal>);
ModelCoefficients::Ptr coefficients_planePtr (new ModelCoefficients), coefficients_cylinderPtr (new ModelCoefficients);
PointIndices::Ptr inliers_planePtr (new PointIndices), inliers_cylinderPtr (new PointIndices);


// Sensor Object(s)
sensor_msgs::PointCloud2 cup;

//Parameter(s)

short int kValue = 50;
int planeModelMaxIteration = 100;
float planeModelDistThresh = 0.03;
float normalDistWeight = 0.1;
int segObjModelMaxIteration = 1000;
float segObjModelDistThresh = 0.05;

