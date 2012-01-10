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

FIND_PACKAGE( YARP REQUIRED )
#FIND_PACKAGE( ICUB REQUIRED )
#INCLUDE_DIRECTORIES( ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${YARP_INCLUDE_DIRS} )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_find_ros_package (al_utils)
#TODO: Add more genious path variables around
#include_directories(${al_utils_PACKAGE_PATH}/include)

rosbuild_add_executable(human_tf_publisher src/human_tf_publisher.cpp)
rosbuild_add_executable(gazebo_st_publisher src/gazebo_st_publisher.cpp)
#rosbuild_add_executable(${PROJECT_NAME} src/main.cpp)
#rosbuild_add_executable(human_js_pub src/human_js_pub.cpp)
#rosbuild_add_executable(human_rs_pub src/human_rs_pub.cpp)
#rosbuild_add_executable(torso_box_st_pub src/torso_box_st_pub.cpp)
#rosbuild_add_executable(torso_shoulders_box_st_pub src/torso_shoulders_box_st_pub.cpp)
#find_library(AL_UTILS al_utils ${al_utils_PACKAGE_PATH}/lib)
#target_link_libraries(human_js_pub ${AL_UTILS})

#if(YARP_FOUND AND ICUB_FOUND)
if(YARP_FOUND)
    INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
    rosbuild_add_executable(icub_js_pub src/icub_js_pub.cpp src/iCub.cpp)
    TARGET_LINK_LIBRARIES(icub_js_pub ${YARP_LIBRARIES})
endif(statement)
