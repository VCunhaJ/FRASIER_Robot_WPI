/*
 *  normal_components.cpp
 *
 *  Created on: Dec 16, 2014
 *      Author: Vanderlei Cunha Jr.
 *
 *
 */
#include<Fixtures/virtual_fixtures.h>
#include<Fixtures/Images/normal_components.h>

using namespace std;
using namespace ros;
using namespace pcl;


void NormalComponents(const sensor_msgs::PointCloud2ConstPtr &object)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("intraoperative_image.pcd", *cloud_RGB) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file intraoperative_image.pcd \n");
    }

    pcl::copyPointCloud(*cloud_RGB, *cloud);

    std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.05);
    ne.compute (*cloud_normals1);

    pcl::visualization::PCLVisualizer viewer ("Normal Components");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 255, 0); // Orange
    viewer.addPointCloud (cloud, cloud_color_handler, "original_cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals1, 10, 0.05, "normals");

    while (!viewer.wasStopped()){
        viewer.spinOnce();
    }
}
int main (int argc, char** argv)
{
    ros::init(argc, argv, "normal_components_node");
    ros::NodeHandle node;


    subscribeSegmentedObject = node.subscribe("/softkinetic_camera/depth/points",1, &NormalComponents);

    Rate loop_Rate(10);

    for(int i=0;i<20;i++){
        loop_Rate.sleep();
    }

    if(node.ok())
    {
        spinOnce();
        loop_Rate.sleep();

    }
    return 0;

}
