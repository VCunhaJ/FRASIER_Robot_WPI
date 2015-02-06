#include<Fixtures/virtual_fixtures.h>


void ransacSeg(const sensor_msgs::PointCloud2 &planeCloud){


	//Convert PointCloud2 msg to PointCloud class object
	pcl::fromROSMsg(planeCloud, *planes_cloud); 

	//RANSAC objects: model and algorithm
	pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(planes_cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZRGB>ransac(model);

	//std::cerr<<"Total points: "<<planes_cloud->width*planes_cloud->height<<std::endl;

	//Set the maximum allowed distance to the model
	ransac.setDistanceThreshold(dist_threshold);
	ransac.computeModel();
	ransac.setMaxIterations(max_iteration);



	std::vector<int> inlierIndices;


	ransac.getInliers(inlierIndices); //return the best set of inliers found so far for this model


	//Copy all inliers of the model to another cloud
	pcl::copyPointCloud<pcl::PointXYZRGB>(*planes_cloud, inlierIndices, *inlierPoints);


	//Convert the PointCloud class object to a PointCloud2 msg
	pcl::toROSMsg(*inlierPoints, finalCloud);


	pub_ransac.publish(finalCloud);




	}


	int main(int argc, char** argv){

	ros::init(argc, argv, "ransac_seg_node");
	ros::NodeHandle node;

	//Subscribe to planar segmentation
	sub_seg = node.subscribe("Planar_Segmentation",1, &ransacSeg);
	//Publish RANSAC segmentations
	pub_ransac = node.advertise<sensor_msgs::PointCloud2>("RANSAC_Segmentation", 1);



	if(ransac_node_shutdown){
		ros::shutdown();
		}
	while(1){

	ros::spin();

	}



}





