/*
 * point_to_point_registration.cpp
 *
 *  Created on: Dec 7, 2014
 *      Author: Vanderlei Cunha Jr.
 */

#include<Fixtures/Registrations/cloud_to_cloud_registration.h>
using namespace std;
using namespace pcl;
using namespace ros;

class FeatureCloud
{
public:
	// A bit of shorthand
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
	typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
	typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

	FeatureCloud () :
			search_method_xyz_ (new SearchMethod),
			normal_radius_ (0.02f),
			feature_radius_ (0.02f)
	{}

	~FeatureCloud () {}

	// Process the given cloud
	void setInputCloud (PointCloud::Ptr xyz)
	{
		xyz_ = xyz;
		processInput ();
	}

	// Load and process the cloud in the given PCD file
	void loadInputCloud (const std::string &pcd_file)
	{
		xyz_ = PointCloud::Ptr (new PointCloud);
		pcl::io::loadPCDFile (pcd_file, *xyz_);
		processInput ();
	}

	// Get a pointer to the cloud 3D points
	PointCloud::Ptr getPointCloud () const
	{
		return (xyz_);
	}

	// Get a pointer to the cloud of 3D surface normals
	SurfaceNormals::Ptr getSurfaceNormals () const
	{
		return (normals_);
	}

	// Get a pointer to the cloud of feature descriptors
	LocalFeatures::Ptr getLocalFeatures () const
	{
		return (features_);
	}

protected:
	// Compute the surface normals and local features
	void processInput ()
	{
		computeSurfaceNormals ();
		computeLocalFeatures ();
	}

	// Compute the surface normals
	void computeSurfaceNormals ()
	{
		normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
		norm_est.setInputCloud (xyz_);
		norm_est.setSearchMethod (search_method_xyz_);
		norm_est.setRadiusSearch (normal_radius_);
		norm_est.compute (*normals_);
	}

	// Compute the local feature descriptors
	void computeLocalFeatures ()
	{
		features_ = LocalFeatures::Ptr (new LocalFeatures);

		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
		fpfh_est.setInputCloud (xyz_);
		fpfh_est.setInputNormals (normals_);
		fpfh_est.setSearchMethod (search_method_xyz_);
		fpfh_est.setRadiusSearch (feature_radius_);
		fpfh_est.compute (*features_);
	}

private:
	// Point cloud data
	PointCloud::Ptr xyz_;
	SurfaceNormals::Ptr normals_;
	LocalFeatures::Ptr features_;
	SearchMethod::Ptr search_method_xyz_;

	// Parameters
	float normal_radius_;
	float feature_radius_;
};

class TemplateAlignment
{
public:

	// A struct for storing alignment results
	struct Result
	{
		float fitness_score;
		Eigen::Matrix4f final_transformation;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	TemplateAlignment () :
			min_sample_distance_ (0.05f),
			max_correspondence_distance_ (0.01f*0.01f),
			nr_iterations_ (500)
	{
		// Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
		sac_ia_.setMinSampleDistance (min_sample_distance_);
		sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
		sac_ia_.setMaximumIterations (nr_iterations_);
	}

	~TemplateAlignment () {}

	// Set the given cloud as the target to which the templates will be aligned
	void
	setTargetCloud (FeatureCloud &target_cloud)
	{
		target_ = target_cloud;
		sac_ia_.setInputTarget (target_cloud.getPointCloud ());
		sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
	}

	// Add the given cloud to the list of template clouds
	void
	addTemplateCloud (FeatureCloud &template_cloud)
	{
		templates_.push_back (template_cloud);
	}

	// Align the given template cloud to the target specified by setTargetCloud ()
	void
	align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
	{
		sac_ia_.setInputCloud (template_cloud.getPointCloud ());
		sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

		pcl::PointCloud<pcl::PointXYZ> registration_output;
		sac_ia_.align (registration_output);

		result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
		result.final_transformation = sac_ia_.getFinalTransformation ();
	}

	// Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
	void
	alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
	{
		results.resize (templates_.size ());
		for (size_t i = 0; i < templates_.size (); ++i)
		{
			align (templates_[i], results[i]);
		}
	}

	// Align all of template clouds to the target cloud to find the one with best alignment score
	int
	findBestAlignment (TemplateAlignment::Result &result)
	{
		// Align all of the templates to the target cloud
		std::vector<Result, Eigen::aligned_allocator<Result> > results;
		alignAll (results);

		// Find the template with the best (lowest) fitness score
		float lowest_score = std::numeric_limits<float>::infinity ();
		int best_template = 0;
		for (size_t i = 0; i < results.size (); ++i)
		{
			const Result &r = results[i];
			if (r.fitness_score < lowest_score)
			{
				lowest_score = r.fitness_score;
				best_template = (int) i;
			}
		}

		// Output the best alignment
		result = results[best_template];
		return (best_template);
	}

private:
	// A list of template clouds and the target to which they will be aligned
	std::vector<FeatureCloud> templates_;
	FeatureCloud target_;

	// The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
	float min_sample_distance_;
	float max_correspondence_distance_;
	int nr_iterations_;
};

void PointToPoint(const sensor_msgs::PointCloud2ConstPtr &object) {
	// Load the object templates specified in the object_templates.txt file
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_RGB(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud(new pcl::PointCloud <pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("preoperative_image_1.pcd", *template_cloud_RGB) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file preoperative_image.pcd \n");
	}

	pcl::copyPointCloud(*template_cloud_RGB, *template_cloud);

	FeatureCloud template_feature;
	template_feature.setInputCloud(template_cloud);

	// Load the target cloud PCD file
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_RGB(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud <pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("intraoperative_image.pcd", *target_cloud_RGB) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file preoperative_image.pcd \n");
	}
	pcl::copyPointCloud(*target_cloud_RGB, *target_cloud);

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
	pcl::PointCloud <pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud <pcl::PointXYZ>);
	transformPointCloud(*best_template.getPointCloud(), *transformed_cloud, best_alignment.final_transformation);
	pcl::io::savePCDFileASCII("transformed_preoperative.pcd", *transformed_cloud);
	std::cerr << "Saved" << std::endl;

	//pcl::visualization::CloudViewer viewPreoperativeImage("Transformed_Pre_Operative Image");

	pcl::visualization::PCLVisualizer viewer ("All Images");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> template_cloud_color_handler (template_cloud, 255, 255, 0); // Orange
	viewer.addPointCloud (template_cloud, template_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color_handler (target_cloud, 0, 0, 255); // White
	viewer.addPointCloud (target_cloud, target_cloud_color_handler, "target_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 255, 0, 0); // Red
	viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce ();
	}
}

// Align a collection of object templates to a sample point cloud
int main (int argc, char **argv)
{
	ros::init(argc, argv, "cloud_to_cloud_registration_node");
	ros::NodeHandle node;


	subscribeSegmentedObject = node.subscribe("/softkinetic_camera/depth/points",1, &PointToPoint);

	ros::Rate loop_Rate(10);

	for(int i=0;i<20;i++){
		loop_Rate.sleep();
	}

	if(node.ok())
	{
		ros::spinOnce();
		loop_Rate.sleep();

	}
	return 0;
}



