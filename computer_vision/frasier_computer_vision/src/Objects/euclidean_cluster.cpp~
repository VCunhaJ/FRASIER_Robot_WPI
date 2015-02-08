/*
 * Author(s): Vanderlei Cunha Jr.
 * PARbot MQP
 * Created December 1, 2014
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
#include<vector>

using namespace std;
using namespace ros;
using namespace pcl;



void EuclideanCallback (const sensor_msgs::PointCloud2& objects)
{


//Convert from PointCloud2 msg to PointCloud class object
 pcl::fromROSMsg(objects, *segmentedObjectsPtr);
 
 kdTreePtr->setInputCloud(segmentedObjectsPtr);
 
 euclideanExtraction.setClusterTolerance(clusterTolerance);
 euclideanExtraction.setMinClusterSize(minSize);
 euclideanExtraction.setMaxClusterSize(maxSize);
 euclideanExtraction.setSearchMethod(kdTreePtr);
 euclideanExtraction.setInputCloud(segmentedObjectsPtr);
 euclideanExtraction.extract(clusterIndices);
 
 for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
 {
    for(pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    clustersPtr->points.push_back(segmentedObjectsPtr->points[*pit]);
    clustersPtr->width = clustersPtr->points.size();
    clustersPtr->height = 1;
    clustersPtr->is_dense = true;
    
    cout<<"Points in Cluster: "<<clustersPtr->points.size()<<endl;
    stream<<"Number of Clusters: "<<i<<endl;
    i++;
 }


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
