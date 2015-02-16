/*
 * Author(s): Vanderlei Cunha Jr., Kristina Walker
 * PARbot MQP
 * Created December 1, 2014
 *
 * @node: surface_reconstruction_node
 * @Publisher: 
 * @Subscriber:
 * @msg: sensor_msgs::PointCloud2
 *
 * @purpose: iterates through all points and interpolates the data, trying to guess how the original surface was
 *
 */
#include<FRASIER/frasier_main.h>
#include<FRASIER/Objects/surface_reconstruction.h>


using namespace std;
using namespace ros;
using namespace pcl;


void SurfaceReconstructionCallBack (const sensor_msgs::PointCloud2& cloud)
{

//Convert from PointCloud2 msg to PointCloud class object
fromROSMsg(cloud, *segmentedObjectsPtr);

reconstructFilter.setInputCloud(segmentedObjectsPtr);
reconstructFilter.setSearchRadius(neighborRadius);
reconstructFilter.setPolynomialFit(true);
reconstructFilter.setComputeNormals(true);
reconstructFilter.setSearchMethod(kdTreePtr);
reconstructFilter.process(*smoothSurfacePtr);

//Convert from PointCloud class to PointCloud2 sensor message
toROSMsg(*smoothSurfacePtr,objectReconstructed );


pubSmoothedSurfaces.publish(objectReconstructed);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "surface_reconstruction_node");
  ros::NodeHandle node;

  
  subSegmentedObjects = node.subscribe("/FRASIER/Fixture/RadiusConditionalFilter",1, &SurfaceReconstructionCallBack);
  pubSmoothedSurfaces = node.advertise<sensor_msgs::PointCloud2>("/FRASIER/Objects/SurfaceReconstruction",1);  

Rate loopRate(10);
 while(node.ok()){
     spin();
     loopRate.sleep();
   }
 
   return 0;
}
