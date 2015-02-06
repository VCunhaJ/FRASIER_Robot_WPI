/*
 * fixed_base_tool.cpp
 *
 *  Created on: Dec 12, 2014
 *      Author: Vanderlei Cunha Jr.
 *
 *  This node is simply two dummy Pointbase2 message type XYZ points in a workspace
 *  to represent represent the pose of the base of the PSM and the tool tip
 */
#include<Fixtures/virtual_fixtures.h>
#include <Fixtures/Images/fixed_base_tool.h>
#include <Fixtures/Segmentations/object_seg.h>


using namespace std;
using namespace ros;
using namespace pcl;


void BaseTipLocation(){


	baseTool->width  = 4;
	baseTool->height = 1;
	baseTool->points.resize (baseTool->width * baseTool->height);


    baseTool->points[0].x = -0.4;
	baseTool->points[0].y = -0.4;
	baseTool->points[0].z = 1.0;

	baseTool->points[1].x = 0.1;
	baseTool->points[1].y = 0.1;
	baseTool->points[1].z = 1.0;
/*


	baseTool->points[3].x = -0.3;
	baseTool->points[3].y = -0.3;
	baseTool->points[3].z = 1.0;

*/

// Executing the transformation
// You can either apply transform_1 or transform_2; they are the same
//pcl::transformPointCloud (*source_cloud, *transf_base_tool, transform_2);
// Visualization
	  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
	      "                        red  = transformed point cloud\n");
	  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");















	visualization::CloudViewer viewBasePosition("Base Point");
	viewBasePosition.showCloud(baseTool);
	toROSMsg(*baseTool,baseToolPoint);

		while (!viewBasePosition.wasStopped()){


			publish_baseTipLocation.publish(baseToolPoint);
		    boost::this_thread::sleep (boost::posix_time::microseconds (100));
		    //spin();
		}
}


/*---Base to Tool Tip Transformation---*/
void FixedBaseToolTransformation()
{

}

/*------------MAIN-----------------------------*/
int main (int argc, char** argv)
{

ros::init(argc, argv, "fixed_base_tool_node");
ros::NodeHandle node;

publish_baseTipLocation = node.advertise<sensor_msgs::PointCloud2>("/daVinci/Fixture/Constants/BaseTipLocation",1);



Rate loopRate(10);
while(node.ok())
{
	BaseTipLocation();
    ros::spin();
    loopRate.sleep();
}

return 0;
}
