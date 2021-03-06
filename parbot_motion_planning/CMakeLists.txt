cmake_minimum_required(VERSION 2.8.3)
project(parbot_motion_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs tf nav_msgs message_generation std_msgs parbot_ui cliff_detection)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 Column.msg
 cart_point.msg
 cost_point.msg
 cost_map.msg
 cost_map_2.msg
 CostPointStamped.msg
 CostMapStamped.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

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
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(motion_planning_utils
  src/motion_planning_utils.cpp
)
target_link_libraries(motion_planning_utils ${catkin_LIBRARIES})

add_library(cost_footprint src/cost_footprint.cpp)
target_link_libraries(cost_footprint ${catkin_LIBRARIES})

add_library(Map_Data src/Map_Data.cpp)
target_link_libraries(Map_Data ${catkin_LIBRARIES})

add_library(cost_mapper src/cost_mapper.cpp)
target_link_libraries(cost_mapper Map_Data cost_footprint ${catkin_LIBRARIES})

add_library(tentacle
  src/tentacle.cpp
)
target_link_libraries(tentacle motion_planning_utils cost_mapper ${catkin_LIBRARIES})

add_library(robot_footprint
  src/robot_footprint.cpp
)
target_link_libraries(robot_footprint ${catkin_LIBRARIES})

add_library(velocity_set
  src/velocity_set.cpp
)
target_link_libraries(velocity_set tentacle robot_footprint ${catkin_LIBRARIES})

add_library(Cliff_Handler src/Cliff_Handler.cpp)
target_link_libraries(Cliff_Handler ${catkin_LIBRARIES})

add_library(laserscan_handler
  src/laserscan_handler.cpp
)
target_link_libraries(laserscan_handler ${catkin_LIBRARIES})

add_library(path_handler
  src/path_handler.cpp
)
target_link_libraries(path_handler ${catkin_LIBRARIES})

add_library(publisher_handler
  src/publisher_handler.cpp
)
target_link_libraries(publisher_handler Cliff_Handler UI_Command_handler ${catkin_LIBRARIES})

add_library(RecoveryTentacle src/RecoveryTentacle.cpp)
target_link_libraries(RecoveryTentacle tentacle path_handler ${catkin_LIBRARIES})

add_library(RecoverySet src/RecoverySet.cpp)
target_link_libraries(RecoverySet RecoveryTentacle tentacle path_handler ${catkin_LIBRARIES})

add_library(Recovery src/Recovery.cpp)
target_link_libraries(Recovery RecoverySet RecoveryTentacle tentacle path_handler ${catkin_LIBRARIES})

add_library(cost_map src/cost_map_node.cpp)
target_link_libraries(cost_map ${catkin_LIBRARIES})

add_library(path_publisher src/path_publisher.cpp)
target_link_libraries(path_publisher ${catkin_LIBRARIES})

add_library(motion_planner src/motion_planner.cpp)
target_link_libraries(motion_planner velocity_set cost_mapper path_publisher ${catkin_LIBRARIES})

add_library(Map_Shrinker src/Map_Shrinker.cpp)
target_link_libraries(Map_Shrinker ${catkin_LIBRARIESf})

add_library(Square_Grid_Footprint src/Square_Grid_Footprint.cpp)
target_link_libraries(Square_Grid_Footprint ${catkin_LIBRARIES})

add_library(Map_Handler src/Map_Handler.cpp)
target_link_libraries(Map_Handler ${catkin_LIBRARIES})

add_library(Pose_TF src/Pose_TF.cpp)
target_link_libraries(Pose_TF ${catkin_LIBRARIES})

add_library(Path_TF src/Path_TF.cpp)
target_link_libraries(Path_TF Pose_TF ${catkin_LIBRARIES})


## Declare a cpp executable
add_executable(parbot_motion_planning_test src/motion_planning.cpp)
add_executable(parbot_motion_planning src/motion_planner.cpp)
add_executable(parbot_timing_demo src/ros_timing.cpp)
add_executable(path_transformer src/path_transformer.cpp)
add_executable(braking_test src/braking_test.cpp)
add_executable(cost_map_generator src/cost_map_node.cpp)
add_executable(Local_Map_Node src/Local_Map_Node.cpp)
add_executable(QR_Pose_TF src/QR_Pose_TF.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(parbot_motion_planning parbot_motion_planning_generate_messages_cpp)
add_dependencies(Map_Shrinker parbot_motion_planning_generate_messages_cpp)
add_dependencies(Cliff_Handler cliff_detection_generate_messages_cpp)
add_dependencies(motion_planning_utils parbot_motion_planning_generate_messages_cpp cliff_detection_generate_messages_cpp roboteq_mc_nxtgen_driver_generate_messages_cpp)
add_dependencies(cost_mapper parbot_motion_planning_generate_messages_cpp)
add_dependencies(path_handler parbot_motion_planning_generate_messages_cpp)
add_dependencies(Cliff_Handler UI_Command_handler)
add_dependencies(cost_map_generator cost_mapper Map_Data parbot_motion_planning_generate_messages_cpp
	roboteq_mc_nxtgen_driver_generate_messages_cpp)
add_dependencies(cost_map parbot_motion_planning_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(parbot_motion_planning_test motion_planner motion_planning_utils tentacle robot_footprint velocity_set laserscan_handler path_handler Map_Handler path_publisher publisher_handler cost_mapper Recovery ${catkin_LIBRARIES})

 target_link_libraries(parbot_motion_planning motion_planning_utils tentacle robot_footprint velocity_set laserscan_handler path_handler publisher_handler cost_mapper Map_Handler path_publisher Recovery RecoveryTentacle ${catkin_LIBRARIES})

target_link_libraries(parbot_timing_demo ${catkin_LIBRARIES})

target_link_libraries(path_transformer ${catkin_LIBRARIES})

target_link_libraries(braking_test publisher_handler cost_mapper tentacle robot_footprint ${catkin_LIBRARIES})

target_link_libraries(cost_map_generator cost_mapper motion_planning_utils ${catkin_LIBRARIES})

target_link_libraries(Local_Map_Node Map_Shrinker ${catkin_LIBRARIES})

target_link_libraries(QR_Pose_TF Pose_TF ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS parbot_motion_planning parbot_motion_planning_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_parbot_motion_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
