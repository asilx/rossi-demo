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
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(PROJECTNAME behavior_manager_asil)

PROJECT(${PROJECTNAME})

#SET(folder_header include/BehaviorModule.h)
#SET(folder_source src/BehaviorModule.cpp src/main.cpp)

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)

#SOURCE_GROUP("Header Files" FILES ${folder_header})
#SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${actionPrimitives_INCLUDE_DIRS}
                    ${perceptiveModels_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_executable(${PROJECT_NAME} src/main.cpp src/BehaviorModule.cpp src/DataLogger.cpp)

TARGET_LINK_LIBRARIES(${PROJECTNAME} actionPrimitives
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

########## application
set(appname ${PROJECTNAME})

set(iCubHelpers_DIR $ENV{ICUB_DIR}/conf)
#MESSAGE(${iCubHelpers_DIR})
find_package(iCubHelpers REQUIRED)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

icub_app_install(${appname} FILES ${conf} DESTINATION conf)
