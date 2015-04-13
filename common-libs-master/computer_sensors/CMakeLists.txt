cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include(CheckIncludeFiles)
check_include_files(sensors/sensors.h HAVE_SENSORS_H)
if(NOT HAVE_SENSORS_H)
	message(AUTHOR_WARNING "Did not detect libsensors header files, you may need to run\nsudo apt-get install libsensors4-dev")
endif(NOT HAVE_SENSORS_H)

rosbuild_add_library(sensor_types src/sensor_types.cpp)

rosbuild_add_executable(sensor_monitor src/sensor_monitor.cpp)
target_link_libraries(sensor_monitor sensors)
target_link_libraries(sensor_monitor sensor_types)
