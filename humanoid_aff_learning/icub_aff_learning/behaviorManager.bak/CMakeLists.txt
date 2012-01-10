# Copyright: (C) 2010 RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

SET(PROJECTNAME behaviorManager)

PROJECT(${PROJECTNAME})

SET(folder_header include/BehaviorModule.h)
SET(folder_source src/BehaviorModule.cpp
		  src/main.cpp)

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${actionPrimitives_INCLUDE_DIRS}
                    ${perceptiveModels_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

#ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})
rosbuild_add_executable(${PROJECT_NAME} src/main.cpp ${folder_source})

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