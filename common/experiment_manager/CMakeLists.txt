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

find_package(YARP REQUIRED)
rosbuild_find_ros_package(behavior_manager)
rosbuild_find_ros_package(al_srvs)

include_directories(${YARP_INCLUDE_DIRS} ${behavior_manager}_PACKAGE_PATH/srv_gen/cpp/include ${al_srvs}_PACKAGE_PATH/srv_gen/cpp/include)


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

rosbuild_add_executable(${PROJECT_NAME} src/main2.cpp)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

rosbuild_add_executable(test_perceptors src/test_perceptors.cpp)
target_link_libraries(test_perceptors ${YARP_LIBRARIES})

rosbuild_add_executable(test_look src/test_look.cpp)
target_link_libraries(test_look ${YARP_LIBRARIES})

rosbuild_add_executable(aff_labels_viz src/aff_labels_viz.cpp)
target_link_libraries(aff_labels_viz ${YARP_LIBRARIES})

#target_link_libraries(example ${PROJECT_NAME})
