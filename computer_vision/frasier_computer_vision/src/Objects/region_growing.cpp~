/*
 * Author(s): Vanderlei Cunha Jr.
 * FRASIER MQP
 * Created January 10, 2014
 *
 * @node: region_growing_seg_node
 * @Publisher: TERMIMAL OUTPUT
 * @Subscriber: /FRASIER/Fixture/RadiusConditionalFilter
 * @msg: sensor_msgs::PointCloud2
 *
 * @purpose: This node calculates the surface normals of each of the clusters (objects) from the surrounding point neighborhood support of the point (k-neighborhood)
 *
 */
#include<FRASIER/frasier_main.h>
#include<FRASIER/Objects/region_growing.h>


using namespace std;
using namespace ros;
using namespace pcl;


/*Function calculates the normal of the clusters*/
/*
void NormalEstimator(PointCloud<PointXYZ>::Ptr cloud)
{
//Local Normal Object
PointCloud <Normal>::Ptr localNormalPtr (new PointCloud <Normal>);

//Initiate Normal Estimation
normalEstimator.setSearchMethod(treePtr);//method is kd-tree (d = 2.5D)
normalEstimator.setInputCloud(cloud);
normalEstimator.setKSearch(kNeighborhood);
normalEstimator.compute (*localNormalPtr);
}

*/

/*Callback Function*/
void RegionGrowingCallback(const sensor_msgs::PointCloud2& objects)
{


//Convert from PointCloud2 msg to PointCloud class object
pcl::fromROSMsg(objects, *mainCloudPtr);
 
normalEstimator.setSearchMethod(treePtr);//method is kd-tree (d = 2.5D)
normalEstimator.setInputCloud(mainCloudPtr);
normalEstimator.setKSearch(kNeighborhood);
normalEstimator.compute (*normalPtr);
 
 
//Initiate Region Growing Algorithm
regionGrowing.setMinClusterSize (minClusterSize);
regionGrowing.setMaxClusterSize (maxClusterSize);
regionGrowing.setSearchMethod (treePtr);
regionGrowing.setNumberOfNeighbours (numNeighbours);
regionGrowing.setInputCloud (mainCloudPtr);
//reg.setIndices (indices);
regionGrowing.setInputNormals (normalPtr);
regionGrowing.setSmoothnessThreshold (smoothThresh);
regionGrowing.setCurvatureThreshold (curvatureThresh);
  
//Extract Clusters and place them in vector
regionGrowing.extract (numClusters);
  
//Display Cloud and Cluster on Terminal
cerr << "Number of clusters is equal to " << numClusters.size () << std::endl;



//cerr << "First cluster has " << numClusters[0].indices.size () << " points." << endl;
//cout << "These are the indices of the points of the initial" <<
//std::endl << "cloud that belong to the first cluster:" << std::endl;
 
}

/*Main Function*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "region_growing_seg_node");
  ros::NodeHandle node;

  
  subSegmentedObjects = node.subscribe("/FRASIER/Fixture/RadiusConditionalFilter",1, &RegionGrowingCallback);
    

Rate loopRate(10);

if(!node.ok())
{
node.shutdown();
}

 while(node.ok()){
     spin();
     loopRate.sleep();
   }
 
 
   return 0;
}
