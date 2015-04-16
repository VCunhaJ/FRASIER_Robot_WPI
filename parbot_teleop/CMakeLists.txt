cmake_minimum_required(VERSION 2.8.3)
project(parbot_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  parbot_ui
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES UI_Command_handler
   CATKIN_DEPENDS joy roboteq_mc_nxtgen_driver roscpp
#  DEPENDS system_lib
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
# add_library(parbot_teleop
#   src/${PROJECT_NAME}/parbot_teleop.cpp
# )

## Declare a cpp executable
add_executable(parbot_teleop_node src/teleop_nxt.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(parbot_teleop_node roboteq_mc_nxtgen_driver_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(parbot_teleop_node
   ${catkin_LIBRARIES}
   UI_Command_handler
 )


