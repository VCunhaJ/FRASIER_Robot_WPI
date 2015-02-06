/*
 *
 * Author(s): Vanderlei Cunha Jr.
 * PARbot MQP
 * Created December 10, 2014
 *
 * @node: passthrough_filter_node
 * @Publisher: daVinci/Fixture/PassthroughFilter
 * @Subscriber: /softkinetic_camera/depth/points
 * @msg: sensor_msgs::PointCloud2
 *
 * @purpose: iterates through cloud once, filters out non-finite points and points outside intervals (0,1.5) on the 
 *           "z" field
 *      
 *
 */
#include <Fixtures/virtual_fixtures.h>
#include <Fixtures/Filters/passthrough_filter.h>

using namespace std;
using namespace pcl;
using namespace std;

void PassFilterCallBack(const sensor_msgs::PointCloud2 &rawcloud)
{

fromROSMsg(rawcloud, *notFilteredCloudPtr);



filterCloud.setInputCloud(notFilteredCloudPtr);
filterCloud.setFilterFieldName("z");
filterCloud.setFilterLimits(0, 1.5);
filterCloud.filter(*passFilteredCloudPtr);

//Convert from PointCloud class to PointCloud2 msg
toROSMsg(*passFilteredCloudPtr, filteredCloudMsg);

publish_filter.publish(filteredCloudMsg);

}

int main (int argc, char** argv)
{

ros::init(argc, argv, "passthrough_filter_node");
ros::NodeHandle node;
subscribe_raw_data = node.subscribe("/softkinetic_camera/depth/points", 1, &PassFilterCallBack);
publish_filter = node.advertise<sensor_msgs::PointCloud2>("/daVinci/Fixture/PassthroughFilter",1);


ros::Rate loopRate(10);

while(ros::ok())
{
ros::spin();
loopRate.sleep();

}


return EXIT_SUCCESS;
}



