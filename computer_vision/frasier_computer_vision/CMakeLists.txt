cmake_minimum_required(VERSION 2.8.3)
project(frasier_computer_vision)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs pcl_ros cv_bridge image_transport camera_info_manager nav_msgs openslam_gmapping rostest tf)

# find the DepthSense SDK
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(DepthSenseSDK)
find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED signals)
find_package(OpenCV)

if(NOT DepthSenseSDK_FOUND)
  message(FATAL_ERROR "+++++++++ Could not locate the DepthSense SDK +++++++++
Please install the SDK before trying to build the 'softkinetic_camera' package, see README.txt.

Cannot continue, aborting.")
  return()
endif()


#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs pcl_ros cv_bridge camera_info_manager
)
###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DepthSenseSDK_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})


#Feedbacks Executable
add_executable(softkinetic_bringup_node src/Feedback/softkinetic_start.cpp)

#Filters Executable
add_executable(passthrough_filter_node src/Filters/passthrough_filter.cpp)
add_executable(radius_conditional_filter_node src/Filters/radius_conditional_filter.cpp)

#Fixtures Executable
add_executable(concave_hull_node src/Fixtures/concave_hull.cpp)

#Images Executable

#add_executable(fixed_base_tool_node src/Images/fixed_base_tool.cpp)
#add_executable(normal_components_node src/Images/normal_components.cpp)


#Registrations Executable
#add_executable(pre_operative_image_node src/Registrations/pre_operative_image.cpp)
#add_executable(intraoperative_image_node src/Registrations/intraoperative_image.cpp)
#add_executable(cloud_to_cloud_registration_node src/Registrations/cloud_to_cloud_registration.cpp)
#add_executable(real_time_ctc_node src/Registrations/real_time_ctc.cpp)

#Segmentations Executable
add_executable(outlier_display_node src/Segmentations/outlier_segmentation.cpp)
#add_executable(object_seg_node src/Segmentations/object_seg.cpp)

#Objects Executable
add_executable(euclidean_cluster_node src/Objects/euclidean_cluster.cpp)
add_executable(region_growing_seg_node src/Objects/region_growing.cpp)


#Tracking Executable


#SLAM Executable




#Feedbacks Target
#target_link_libraries(extract_borders_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(softkinetic_bringup_node ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})

#Filters Target
target_link_libraries(passthrough_filter_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(radius_conditional_filter_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})

#Fixtures Target
target_link_libraries(concave_hull_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})

#Images Target
#target_link_libraries(normal_components_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})
#target_link_libraries(fixed_base_tool_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})


#Registrations Target
#target_link_libraries(pre_operative_image_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})
#target_link_libraries(intraoperative_image_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})
#target_link_libraries(cloud_to_cloud_registration_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})
#target_link_libraries(real_time_ctc_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})

#Segmentations Target
target_link_libraries(outlier_display_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})
#target_link_libraries(object_seg_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})

#Objects Target
target_link_libraries(euclidean_cluster_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(region_growing_seg_node ${PCL_LIBRARIES} ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})



#SLAM Target




install(TARGETS softkinetic_bringup_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
