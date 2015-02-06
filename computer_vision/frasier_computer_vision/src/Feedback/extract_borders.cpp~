/*
 * extract_borders.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: Vanderlei Cunha Jr.
 *
 *
 */

#include<Fixtures/Feedback/extract_borders.h>
using namespace pcl;
using namespace ros;
using namespace std;

void RangeOutput(const sensor_msgs::PointCloud2 &point)
{
     fromROSMsg(point, *rangePointPtr);

	// Sensor pose. Thankfully, the cloud includes the data.
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(rangePointPtr->sensor_origin_[0],
			rangePointPtr->sensor_origin_[1],
			rangePointPtr->sensor_origin_[2])) *
								 Eigen::Affine3f(rangePointPtr->sensor_orientation_);
	// Noise level. If greater than 0, values of neighboring points will be averaged.
	// This would set the search radius (e.g., 0.03 == 3cm).
	float noiseLevel = 0.0f;
	// Minimum range. If set, any point closer to the sensor than this will be ignored.
	float minimumRange = 0.0f;
	// Border size. If greater than 0, a border of "unobserved" points will be left
	// in the image when it is cropped.
	int borderSize = 1;

	// Range image object.
	RangeImage rangeImage;
	rangeImage.createFromPointCloud(*rangePointPtr, angularResolutionX, angularResolutionY,
									maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::CAMERA_FRAME,
									noiseLevel, minimumRange, borderSize);

	// Visualize the image.
	visualization::RangeImageVisualizer viewer("Range image");
	viewer.showRangeImage(rangeImage);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		// Sleep 100ms to go easy on the CPU.
		pcl_sleep(0.1);
	}


}



int main(int argc, char** argv)
{
	init(argc, argv, "extract_borders_node");
	NodeHandle node;

	sub_object = node.subscribe("/daVinci/Fixture/OutliersDisplay",1, &RangeOutput);
	pub_range = node.advertise<sensor_msgs::PointCloud2>("/daVinci/Feedback/Range",1);

	Rate loop_rate(10);
	while(node.ok())
	{
		spin();
		loop_rate.sleep();

	}


	return 0;
}
