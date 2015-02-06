/*
 * Author(s): Vanderlei Cunha Jr.
 * PARbot MQP
 * Created December 13, 2014
 *
 * @node: object_seg_node
 * @Publisher: 
 * @Subscriber: 
 * @msg: sensor_msgs::PointCloud2
 *
 * @purpose: uses Euclidean Cluster extraction algorithm to define how many objects are on the plane
 *
 *
 */
#include <Fixtures/virtual_fixtures.h>
#include <Fixtures/Segmentations/object_seg.h>

using namespace pcl;
using namespace std;
using namespace ros;

/*
void RadiusFilter(PointCloud<PointXYZRGB>::Ptr notFilteredCloud)
{
radiusFilter.setInputCloud(notFilteredCloudPtr);
radiusFilter.setRadiusSearch(0.12);
radiusFilter.setMinNeighborsInRadius((notFilteredCloudPtr->points.size()/4));
radiusFilter.filter(*radiusFilteredCloudPtr);


}
*/

void EuclideanCluster(PointCloud<PointXYZRGB>::Ptr new_cloud)
{
//RadiusFilter(new_cloud);
kdTree->setInputCloud(new_cloud);
cluster.setClusterTolerance(CLUSTER_TOLERANCE);
cluster.setMinClusterSize(MINIMUM_CLUSTER_SIZE);
cluster.setMaxClusterSize(MAXIMUM_CLUSTER_SIZE);
cluster.setSearchMethod(kdTree);
cluster.setInputCloud(new_cloud);
cluster.extract(clusterIndices);


for(i = clusterIndices.begin(); i!=clusterIndices.end(); ++i){
   for(point = i->indices.begin(); point != i->indices.end(); point++)
						clusterObject->points.push_back(cloud->points[*point]);
						clusterObject->width = clusterObject->points.size();
						clusterObject->height = 1;
						clusterObject->is_dense = true;

  // if(clusterObject->points.size() <= 0)
      // break;

currentCluster++;
   }
int cluster_length = clusterIndices.size();
cerr<<cluster_length<<" Objects Found "<<endl;
}

void segmentationCallBack(const sensor_msgs::PointCloud2 &filteredCloud)
{

fromROSMsg(filteredCloud, *cloud);

EuclideanCluster(cloud);


toROSMsg(*cloud,objectSegmented);
publish_number_of_objects.publish(objectSegmented);
}

int main (int argc, char** argv)
{

ros::init(argc, argv, "object_seg_node");
ros::NodeHandle node;

subscribe_clusters = node.subscribe("/daVinci/Fixture/RadiusConditionalFilter", 1, &segmentationCallBack);

publish_number_of_objects = node.advertise<sensor_msgs::PointCloud2>("/daVinci/Segmentation/ObjectSegmentation",1);

ros::Rate loopRate(10);
while(ros::ok())
{
ros::spin();
loopRate.sleep();

}

return EXIT_SUCCESS;
}



