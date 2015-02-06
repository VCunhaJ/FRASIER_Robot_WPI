daVinci Virtual Fixtures
===================================

General description
---------------------
This package fuses multiple PCL-oriented nodes that utilizes the softkinetic API package via Senz3D Interactive Gesture Camera
in ROS and performs Virtual Fixtures Operations for the PARbot MQP


#### Parameters
**camera_link** *(string, default: "/softkinetic_link")*   
 The frame ID of the camera.

**confidence_threshold** *(int, default: 150)*   
 Confidence threshold for DepthNode configuration.
 Sensor noise is filtered by increasing the threshold.
 Threshold needs to be within [0, 32767].


#### Published Topics

**Feedback/Softkinetic_Bringup** *(sensor_msgs::PointCloud2)*
 1. @node: softkinetic_bringup_node
 2. @Publisher: 
 3. @Subscriber:NONE
 4. @msg: sensor_msgs::PointCloud2
 5. @purpose: API for communicating Senz3D camera with ROS

**Filters/PassThrough** *(sensor_msgs::PointCloud2)*   
 1. @node: passthrough_filter_node
 2. @Publisher: daVinci/Fixture/PassthroughFilter
 3. @Subscriber: /softkinetic_camera/depth/points
 4. @msg: sensor_msgs::PointCloud2
 5. @purpose: iterates through cloud once, filters out non-finite points and points outside intervals (0,1.5) on the "z" field

**Filters/RadiusConditional** *(sensor_msgs::PointCloud2)*   
 1. @node: radius_conditional_filter_node
 2. @Publisher: /daVinci/Fixture/RadiusConditionalFilter
 3. @Subscriber:/daVinci/Segmentation/ObjectSegmentation
 4. @msg: sensor_msgs::PointCloud2
 5. @purpose: eliminates noise from cloud by removing all points outside of a radius treshold  

**Segmentation/OutlierDisplay** *(sensor_msgs::PointCloud2)*   
 1. @node: outlier_display_node
 2. @Publisher: /daVinci/Fixture/OutliersDisplay
 3. @Subscriber: /daVinci/Fixture/PassthroughFilter
 4. @msg: sensor_msgs::PointCloud2
 5. @purpose: finds the biggest plane (wall or table) using RANSAC (SAC_RANSAC) plane model fitting and removes biggest plane and displays the remaining points
     

**Segmentation/Object_Segmentation** *(sensor_msgs::PointCloud2)*   
 1. @node: 
 2. @Publisher: 
 3. @Subscriber:
 4. @msg: sensor_msgs::PointCloud2
 5. @purpose:

**Images/Fixed_Base_Tool** *(sensor_msgs::PointCloud2)*   
 1. @node: fixed_base_tool_node
 2. @Publisher: /daVinci/Fixture/Constants/BaseTipLocation
 3. @Subscriber: NONE
 4. @msg: sensor_msgs::PointCloud2
 5. @purpose: two arbitrary PointCloud2 points in space with a predefined pose - it is used to insinuate transformation between PSM base to Tooltip

**Images/Normal_Components** *(sensor_msgs::PointCloud2)*   
 1. @node: normal_components_node
 2. @Publisher: 
 3. @Subscriber: /softkinetic_camera/depth/points
 4. @msg: sensor_msgs::PointCloud2
 5. @purpose: assigns objects surface point to be Point Type Normal and creates a surface Normal vector field 

**Registration/Preoperative_object** *(sensor_msgs::PointCloud2)*   
 1. @node: pre_operative_image_node
 2. @Publisher: PCD file
 3. @Subscriber:/softkinetic_camera/depth/points
 4. @msg: sensor_msgs::PointCloud2
 5. @purpose: stores a 3D point cloud object in a PCD file

**Registration/Intraoperative_object** *(sensor_msgs::PointCloud2)*   
 1. @node: intra_operative_image_node
 2. @Publisher: PCD file
 3. @Subscriber:/softkinetic_camera/depth/points
 4. @msg: sensor_msgs::PointCloud2
 5. @purpose: stores a 3D point cloud object in a PCD file

**Registration/Cloud_to_Cloud_Registration** *(sensor_msgs::PointCloud2)*   
 1. @node: cloud_to_cloud_registration_node
 2. @Publisher: PCD
 3. @Subscriber: /softkinetic_camera/depth/points
 4. @msg: sensor_msgs::PointCloud2
 5. @purpose: registration of a 3D object by extracting surface points from preoperative and intraoperative clouds and registering them - high register quality

**Data Analysis/** 


#### Launch Files
**virtual_fixtures_display.launch** *(visualization::RViz)*   
  Displays the following nodes on RViz

			1 softkinetic_bringup_node
			2 passthrough_filter_node
			3 fixed_base_tool_node
			4 object_seg_node
			5 outlier_display_nod
			6 reconstruction_triangulation_node
			7 pre_operative_image_node
			8 concave_hull_node

 
 


			

