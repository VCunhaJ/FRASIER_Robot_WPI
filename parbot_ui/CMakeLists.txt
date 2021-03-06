cmake_minimum_required(VERSION 2.8.3)
project(parbot_ui)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  parbot_msgs
  roscpp
  rospy
  message_runtime
  message_generation
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES parbot_ui
  CATKIN_DEPENDS parbot_msgs roscpp rospy
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
  INCLUDE_DIRS include
)

## Declare a cpp library
add_library(ESTOP
  src/Emergency/ESTOP.cpp
)

add_library(UI_Command_handler
  src/Commands/UI_Command_handler.cpp
)

add_library(Command_Pub_Handler
  src/Commands/Command_Pub_Handler.cpp
)


## Declare a cpp executable
add_executable(estop_node src/Emergency/estop_node.cpp)
add_executable(estop_ui_node src/Emergency/estop_ui_node.cpp)
add_executable(Disable_Following src/Tracking/Disable_Following.cpp)
add_executable(Enable_Following src/Tracking/Enable_Following.cpp)
add_executable(Enable_Teleop src/Teleop/Enable_Teleop.cpp)
add_executable(Disable_Teleop src/Teleop/Disable_Teleop.cpp)
add_executable(Resume src/Commands/resume_ui_node.cpp)
add_executable(Enable_Senz3D src/CV/Enable_Senz3D.cpp)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(parbot_ui_node parbot_ui_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(estop_node
  ${catkin_LIBRARIES}
  ESTOP
)

target_link_libraries(estop_ui_node
  ${catkin_LIBRARIES}
  Command_Pub_Handler
)

target_link_libraries(Disable_Following
  ${catkin_LIBRARIES}
  Command_Pub_Handler
)

target_link_libraries(Enable_Following
  ${catkin_LIBRARIES}
  Command_Pub_Handler
)

target_link_libraries(Enable_Teleop
  ${catkin_LIBRARIES}
  Command_Pub_Handler
)

target_link_libraries(Disable_Teleop
  ${catkin_LIBRARIES}
  Command_Pub_Handler
)

target_link_libraries(Resume
  ${catkin_LIBRARIES}
  Command_Pub_Handler
)

target_link_libraries(Enable_Senz3D
  ${catkin_LIBRARIES}
  Command_Pub_Handler
)


