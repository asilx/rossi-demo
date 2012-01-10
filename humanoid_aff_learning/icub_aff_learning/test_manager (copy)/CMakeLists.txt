cmake_minimum_required(VERSION 2.4.6)

set(ROS_BUILD_TYPE Debug)
rosbuild_init()

SET(PROJECTNAME testManager)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp)

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)

SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${actionPrimitives_INCLUDE_DIRS}
                    ${perceptiveModels_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

rosbuild_add_executable(${PROJECTNAME} ${folder_source})
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} actionPrimitives
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

