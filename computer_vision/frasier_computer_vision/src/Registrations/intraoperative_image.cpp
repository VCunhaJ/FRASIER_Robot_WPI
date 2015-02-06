/*
 * intraoperative_image.cpp
 *
 *  Created on: Dec 16, 2014
 *      Author(s): Vanderlei Cunha Jr.
 *                 Christian Kaan
 *                 Jiaqi Ren
 *
 *      Uses a RGBD camera to obtain an intraoperative image of the object
 *
 */



#include<Fixtures/virtual_fixtures.h>
#include<Fixtures/Registrations/intraoperative_image.h>

using namespace std;
using namespace ros;
using namespace pcl;


void IntraoperativeImage(const sensor_msgs::PointCloud2ConstPtr &object)
{

    pcl::fromROSMsg(*object, *segCloudPtr);


    segmentation.setOptimizeCoefficients(true); //enable model coefficient refinement (optional)
    segmentation.setModelType(SACMODEL_PLANE); //configure object to look for a plane
    segmentation.setMethodType(SAC_RANSAC); //use RANSAC method model
    segmentation.setDistanceThreshold(max_thresh); // how close a point must be to the model in order to be considered an inlier.
    segmentation.setMaxIterations(max_iteration);


    int i=0, nr_points= (int) segCloudPtr->points.size();
    while(segCloudPtr->points.size() > 0.3*nr_points) {
        segmentation.setInputCloud(segCloudPtr);
        segmentation.segment(*inlierIndices, *coefficients);

        if (inlierIndices->indices.size() == 0) {
            std::cerr << "Wall Not Found" << std::endl;
            break;
        }

        pcl::ExtractIndices <PointXYZRGB> ex;
        ex.setInputCloud(segCloudPtr);
        ex.setIndices(inlierIndices);
        ex.setNegative(false);
        ex.filter(*planePtr);
        ex.setNegative(true);
        ex.filter(*planeExtractedPtr);
        *segCloudPtr = *planeExtractedPtr;

    }

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // build the filter
    outrem.setInputCloud(segCloudPtr);
    outrem.setRadiusSearch(0.12);
    outrem.setMinNeighborsInRadius ((segCloudPtr->points.size()/4));
    // apply filter
    outrem.filter (*segCloudPtrFiltered);

    io::savePCDFileASCII ("intraoperative_image.pcd", *segCloudPtrFiltered);
    std::cerr << "Saved " << segCloudPtrFiltered->points.size () << " data points to intraoperative_image.pcd.";

    visualization::CloudViewer viewPreoperativeImage("Intraoperative Image");
    while (!viewPreoperativeImage.wasStopped()){
        viewPreoperativeImage.showCloud(segCloudPtrFiltered);
    }

}
int main (int argc, char** argv)
{
    ros::init(argc, argv, "intra_operative_image_node");
    ros::NodeHandle node;


    subscribeSegmentedObject = node.subscribe("/softkinetic_camera/depth/points",1, &IntraoperativeImage);

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

