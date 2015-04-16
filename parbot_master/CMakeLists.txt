cmake_minimum_required(VERSION 2.8.3)
project(parbot_master)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES parbot_master
  CATKIN_DEPENDS roscpp rospy
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(parbot_master
#   src/${PROJECT_NAME}/parbot_master.cpp
# )

## Declare a cpp executable
 add_executable(parbot_sh_test src/parbot_sh_test.cpp)
add_executable(parbot_sh_test_receiver src/parbot_sh_test_receiver.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(parbot_master_node parbot_master_generate_messages_cpp)

# Specify libraries to link a library or executable target against 
target_link_libraries(parbot_sh_test 
${catkin_LIBRARIES}
)
target_link_libraries(parbot_sh_test_receiver 
${catkin_LIBRARIES}
)





