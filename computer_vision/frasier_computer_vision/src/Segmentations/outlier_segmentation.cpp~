/*
 * Author(s): Vanderlei Cunha Jr.
 * PARbot MQP
 * Created December 1, 2014
 *
 * @node: outlier_display_node
 * @Publisher: 
 * @Subscriber:
 * @msg: sensor_msgs::PointCloud2
 *
 * @purpose: finds the biggest plane (wall or table) using RANSAC (SAC_RANSAC) plane model fitting and removes biggest plane 
 *                and displays the remaining points

 *
 */

#include<FRASIER/frasier_main.h>
#include<FRASIER/Segmentations/outlier_segmentation.h>

using namespace std;
using namespace pcl;
using namespace ros;


void VoxelFilter(PointCloud<PointXYZRGB>::Ptr raw, PointCloud<PointXYZRGB>::Ptr cloud)
{
	// Create the filtering object: downsample the dataset using a leaf size of 1cm

VoxelGrid<PointXYZRGB> voxel;
voxel.setInputCloud (needsFilterPtr);
voxel.setLeafSize (0.01f, 0.01f, 0.01f);
voxel.filter(*segCloudPtr);
 std::cout<<" Cloud after Voxel Filter "<<raw->points.size()<<std::endl;

}


void RANSACModel(PointCloud<PointXYZRGB>::Ptr table)
{
SampleConsensusModelPlane<PointXYZRGB>::Ptr model(new SampleConsensusModelPlane<PointXYZRGB>(table));
RandomSampleConsensus<PointXYZRGB> ransac(model);

  //Set the maximum allowed distance to the model
  ransac.setDistanceThreshold(dist_threshold);
  ransac.computeModel();
  ransac.setMaxIterations(iteration_num);

  ransac.getInliers(inlierIndicesVector);

// Copy all inliers of the model to another cloud.
 copyPointCloud<PointXYZRGB>(*table, inlierIndicesVector, *inlierPointsPtr);

}

/*------------------------------------------------------------------------------------------------*/
void outlierDisplay(const sensor_msgs::PointCloud2& msg){




//Convert from PointCloud2 msg to PointCloud class object
 pcl::fromROSMsg(msg, *segCloudPtr);


segmentation.setOptimizeCoefficients(true); //enable model coefficient refinement (optional)
segmentation.setModelType(SACMODEL_PLANE); //configure object to look for a plane
segmentation.setMethodType(SAC_RANSAC); //use RANSAC method model
segmentation.setDistanceThreshold(max_thresh); // how close a point must be to the model in order to be considered an inlier.
segmentation.setMaxIterations(max_iteration);


int i=0, nr_points= (int) segCloudPtr->points.size();
while(segCloudPtr->points.size() > 0.3*nr_points)
{
	segmentation.setInputCloud(segCloudPtr);
	segmentation.segment(*inlierIndices, *coefficients);

	if (inlierIndices->indices.size() == 0)
	{
			std::cerr<<"Surface Not Found"<<std::endl;
			break;
	}

	ExtractIndices<PointXYZRGB> ex;
	ex.setInputCloud(segCloudPtr);
	ex.setIndices(inlierIndices);
	ex.setNegative(false);
	ex.filter(*planePtr);
	ex.setNegative(true);
	ex.filter(*planeExtractedPtr);
	*segCloudPtr = *planeExtractedPtr;

//std::cerr<<"3D Model has: "<<planeExtractedPtr->width*planeExtractedPtr->height<<" data points."<<std::endl;

RANSACModel(segCloudPtr);


//Convert from PointCloud class to PointCloud2 msg
pcl::toROSMsg(*segCloudPtr, pc_data);


pub_seg.publish(pc_data);

}


}
/*------------------------------------------------------------------------------------------------*/

int main (int argc, char** argv){

//Initiate ROS

init(argc, argv,"outlier_display_node");
NodeHandle node;


//Subscribe to raw data topic

sub_raw_data = node.subscribe("/FRASIER/Fixture/PassthroughFilter", 1, &outlierDisplay);
pub_seg = node.advertise<sensor_msgs::PointCloud2>("/FRASIER/Fixture/BigPlaneRemoved", 1);
Rate loopRate(10);
while(node.ok()){

	ros::spin();
	loopRate.sleep();

	}
return 0;
}
