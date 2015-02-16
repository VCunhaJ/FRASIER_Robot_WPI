/*
 * Author(s): Vanderlei Cunha Jr., Kristina Walker
 * PARbot MQP
 * Created February 11, 2014
 *
 * @node: euclidean_cluster_node
 * @Publisher: 
 * @Subscriber:
 * @msg: sensor_msgs::PointCloud2
 *
 * @purpose: create a Kd-Tree representation of the object segmented 
 *
 */
#include<FRASIER/frasier_main.h>
#include<FRASIER/Objects/euclidean_cluster.h>


using namespace std;
using namespace ros;
using namespace pcl;


void EuclideanCallback (const sensor_msgs::PointCloud2& objects)
{


//Convert from PointCloud2 msg to PointCloud class object
 pcl::fromROSMsg(objects, *segmentedObjectsPtr);
 
 kdTreePtr->setInputCloud(segmentedObjectsPtr);
 
 euclideanExtraction.setSearchMethod(kdTreePtr);
 euclideanExtraction.setClusterTolerance(clusterTolerance);
 euclideanExtraction.setMinClusterSize(minSize);
 euclideanExtraction.setMaxClusterSize(maxSize);
 euclideanExtraction.setSearchMethod(kdTreePtr);
 euclideanExtraction.setInputCloud(segmentedObjectsPtr);
 euclideanExtraction.extract(clusterIndices);
 
 int i = 0;
 for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
 {
        
    for(pit = it->indices.begin (); pit != it->indices.end (); pit++)
    
    clustersPtr->points.push_back(segmentedObjectsPtr->points[*pit]);
    clustersPtr->width = clustersPtr->points.size();
    clustersPtr->height = 1;
    clustersPtr->is_dense = true;
    
    clusterDisplay = clustersPtr->points.size();
 //   cerr<<"Points in Cloud: "<<clusterDisplay<<endl;

 i++;
 }
 
 int clusterLength = clusterIndices.size();
    //cerr<<"Number of Clusters: "<<clusterLength<<endl;

    ROS_INFO("Found %lu clusters:", clusterIndices.size());
    
    //Empty Buffer
    clusterIndices.clear();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "euclidean_cluster_node");
  ros::NodeHandle node;

  
  subSegmentedObjects = node.subscribe("/FRASIER/Fixture/RadiusConditionalFilter",1, &EuclideanCallback);
    

Rate loopRate(10);
 while(node.ok()){
     spin();
     loopRate.sleep();
   }
 
 
   return 0;
}
