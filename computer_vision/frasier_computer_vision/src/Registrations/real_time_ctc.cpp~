/*
 * real_time_ctc.cpp
 *
 *  Created on: Dec 20, 2014
 *      Author(s): Vanderlei Cunha Jr.
 *                 Christian Kaan
 *                 Jiaqi Ren
 *
 *      
 */



#include<Fixtures/virtual_fixtures.h>
#include<Fixtures/Registrations/real_time_ctc.h>
#include<Fixtures/Registrations/cloud_to_cloud_registration.h>

using namespace std;
using namespace ros;
using namespace pcl;


void RealTimeCTC(const sensor_msgs::PointCloud2ConstPtr &object)
{

    // First, Obtain the intraoperative image and filter
    // Then, Register the template to that image

    // Grab and filter the point cloud from the stereo vision
    // Change the ROS msg to a PointCloud XYZRGB
    pcl::fromROSMsg(*object, *segCloudPtr);

    segmentation.setOptimizeCoefficients(true); //enable model coefficient refinement (optional)
    segmentation.setModelType(SACMODEL_PLANE); //configure object to look for a plane
    segmentation.setMethodType(SAC_RANSAC); //use RANSAC method model
    segmentation.setDistanceThreshold(max_thresh); // how close a point must be to the model in order to be considered an inlier.
    segmentation.setMaxIterations(max_iteration);

    // Extract the largest plane from the point cloud
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

    // Filter out additional noise by adding a Radius filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(segCloudPtr);

    // These next two values are specific to the item, and should be adjusted to compensate
    outrem.setRadiusSearch(0.12);
    outrem.setMinNeighborsInRadius ((segCloudPtr->points.size()/4));
    // apply filter
    outrem.filter (*segCloudPtrFiltered);



    // Start of the registration
    // Load the object template obtained through preoperative scans
    PointCloud<PointXYZRGB>::Ptr template_cloud_RGB(new PointCloud <PointXYZRGB>);
    PointCloud<PointXYZ>::Ptr template_cloud(new PointCloud <PointXYZ>);
    if (io::loadPCDFile<PointXYZRGB>("preoperative_image_1.pcd", *template_cloud_RGB) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file preoperative_image.pcd \n");
    }
    // Need to convert the object from and Point Cloud XYZRGB to a Point Cloud XYZ
    copyPointCloud(*template_cloud_RGB, *template_cloud);

    // Create a Feature cloud for the template.
    FeatureCloud template_feature;
    template_feature.setInputCloud(template_cloud);

    // Load the target cloud PCD file
    PointCloud<PointXYZ>::Ptr target_cloud(new PointCloud <PointXYZ>);
    // Convert the intraoperative image from XYZRGB to XYZ
    copyPointCloud(*segCloudPtrFiltered, *target_cloud);

    // Create a Feature cloud for the target object.
    FeatureCloud target_feature;
    target_feature.setInputCloud(target_cloud);

    // Set the TemplateAlignment inputs
    TemplateAlignment template_align;
    template_align.addTemplateCloud(template_feature);
    template_align.setTargetCloud(target_feature);

    // Find the best template alignment
    TemplateAlignment::Result best_alignment;
    int best_index = template_align.findBestAlignment(best_alignment);
    const FeatureCloud &best_template = template_feature;

    // Print the alignment fitness score (values less than 0.00002 are good)
    printf("Best fitness score: %f\n", best_alignment.fitness_score);

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);

    printf("\n");
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    printf("\n");
    printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

    // Save the aligned template for visualization
    PointCloud <PointXYZ>::Ptr transformed_cloud(new PointCloud <PointXYZ>);
    transformPointCloud(*best_template.getPointCloud(), *transformed_cloud, best_alignment.final_transformation);
    io::savePCDFileASCII("transformed_preoperative.pcd", *transformed_cloud);
    std::cerr << "Saved" << std::endl;

    // Output a visualization of the preoperative, intraoperative, and transformed clouds
    visualization::PCLVisualizer viewer ("All Images");

    // Add the preoperative image. Color for this cloud is Yellow
    visualization::PointCloudColorHandlerCustom<PointXYZ> template_cloud_color_handler (template_cloud, 255, 255, 0);
    viewer.addPointCloud (template_cloud, template_cloud_color_handler, "original_cloud");

    // Add the intraoperative image. Color for this cloud is Blue
    visualization::PointCloudColorHandlerCustom<PointXYZ> target_cloud_color_handler (target_cloud, 0, 0, 255);
    viewer.addPointCloud (target_cloud, target_cloud_color_handler, "target_cloud");

    // Add the transformed preoperative image. Color for this cloud is Red.
    visualization::PointCloudColorHandlerCustom<PointXYZ> transformed_cloud_color_handler (transformed_cloud, 255, 0, 0);
    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    // Display all the point clouds for 10 secs
    while (!viewer.wasStopped ()) {
        for(int i=0; i<10000; i++) {
            viewer.spinOnce(); // Since each spinOnce goes for 1ms, should need to loop 10000 times for
        }
        viewer.close();
    }
}

// Align a collection of object templates to a sample point cloud
int main (int argc, char **argv)
{
    ros::init(argc, argv, "real_time_ctc_node");
    ros::NodeHandle node;


    subscribeSegmentedObject = node.subscribe("/softkinetic_camera/depth/points",1, &RealTimeCTC);

    ros::Rate loop_Rate(10);
    // Allow the camera to load initially
    for(int i=0;i<20;i++){
        loop_Rate.sleep();
    }

    // Until the program is closed, run it every 20 seconds to update the registration
    while(node.ok())
    {
        ros::spinOnce();
        for(int i=0;i<200;i++) {
            loop_Rate.sleep();
        }

    }
    return 0;
}
