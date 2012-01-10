# Copyright: (C) 2010 RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME actionLib)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${actionPrimitives_INCLUDE_DIRS}
                    ${perceptiveModels_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

MESSAGE("${actionPrimitives_INCLUDE_DIRS}")

TARGET_LINK_LIBRARIES(${PROJECTNAME} actionPrimitives
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

########## application
set(appname ${PROJECTNAME})

set(iCubHelpers_DIR $ENV{ICUB_DIR}/conf)
find_package(iCubHelpers REQUIRED)
icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

icub_app_install(${appname} FILES ${conf} DESTINATION conf)


