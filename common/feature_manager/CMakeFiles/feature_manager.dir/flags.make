# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# compile CXX with /usr/bin/c++
CXX_FLAGS = -O2 -g -I/usr/local/MATLAB/R2011a/extern/include -I/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/common/feature_manager/include -I/opt/ros/electric/stacks/perception_pcl/pcl_ros/include -I/opt/ros/electric/stacks/perception_pcl/pcl_ros/cfg/cpp -I/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/humanoid_aff_learning/icub_aff_learning/tabletop_2D_segmentation/srv_gen/cpp/include -I/opt/ros/electric/stacks/vision_opencv/cv_bridge/include -I/usr/include/opencv-2.3.1/opencv -I/usr/include/opencv-2.3.1 -I/opt/ros/electric/stacks/pr2_object_manipulation/perception/tabletop_object_detector/include -I/opt/ros/electric/stacks/pr2_object_manipulation/perception/tabletop_object_detector/msg_gen/cpp -I/opt/ros/electric/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp -I/opt/ros/electric/stacks/pr2_object_manipulation/perception/tabletop_object_detector/msg_gen/cpp/include -I/opt/ros/electric/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp/include -I/opt/ros/electric/stacks/nodelet_core/nodelet_topic_tools/include -I/opt/ros/electric/stacks/nodelet_core/nodelet/include -I/opt/ros/electric/stacks/nodelet_core/nodelet/srv_gen/cpp/include -I/opt/ros/electric/stacks/pluginlib/include -I/opt/ros/electric/stacks/pluginlib -I/opt/ros/electric/stacks/bond_core/bondcpp/include -I/opt/ros/electric/stacks/bond_core/bond/msg_gen/cpp/include -I/opt/ros/electric/stacks/bond_core/smclib/include -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/include -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/msg/cpp -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/srv/cpp -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/msg_gen/cpp/include -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/srv_gen/cpp/include -I/opt/ros/electric/stacks/perception_pcl/pcl/include/pcl-1.1 -I/usr/include/vtk-5.4 -I/usr/lib/openmpi/include -I/usr/lib/openmpi/include/openmpi -I/usr/include/tcl8.5 -I/usr/include/python2.7 -I/usr/lib/jvm/default-java/include -I/usr/include/libxml2 -I/usr/include/freetype2 -I/opt/ros/electric/stacks/perception_pcl/pcl/msg_gen/cpp/include -I/usr/include/eigen3 -I/opt/ros/electric/stacks/perception_pcl/cminpack/include -I/opt/ros/electric/stacks/perception_pcl/flann/include -I/opt/ros/electric/stacks/motion_planning_control/distance_field/include -I/opt/ros/electric/stacks/common_msgs/visualization_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/motion_planning_control/mapping_msgs/msg/cpp -I/opt/ros/electric/stacks/motion_planning_control/mapping_msgs/srv/cpp -I/opt/ros/electric/stacks/motion_planning_control/mapping_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/motion_planning_control/geometric_shapes_msgs/msg/cpp -I/opt/ros/electric/stacks/motion_planning_control/geometric_shapes_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/object_manipulation/household_objects_database_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/arm_navigation_msgs/include -I/opt/ros/electric/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/arm_navigation_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/actionlib_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/trajectory_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/geometry/tf/include -I/opt/ros/electric/stacks/geometry/tf/msg_gen/cpp/include -I/opt/ros/electric/stacks/geometry/tf/srv_gen/cpp/include -I/opt/ros/electric/stacks/bullet/include -I/opt/ros/electric/stacks/geometry/angles/include -I/opt/ros/electric/stacks/ros_comm/utilities/message_filters/include -I/opt/ros/electric/stacks/common_msgs/sensor_msgs/include -I/opt/ros/electric/stacks/common_msgs/sensor_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/sensor_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/tools/rosbag/include -I/opt/ros/electric/stacks/ros_comm/tools/topic_tools/include -I/opt/ros/electric/stacks/ros_comm/tools/topic_tools/srv_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/tools/rostest/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/msg_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/srv_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/include -I/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/src -I/opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/messages/std_msgs/include -I/opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/cpp/include -I/opt/ros/electric/ros/core/roslib/msg_gen/cpp/include -I/opt/ros/electric/ros/core/roslib/include -I/opt/ros/electric/ros/tools/rospack -I/opt/ros/electric/ros/tools/rospack/include -I/opt/ros/electric/stacks/ros_comm/tools/rosconsole/include -I/opt/ros/electric/stacks/ros_comm/utilities/rostime/include -I/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/include -I/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/common/feature_manager/srv_gen/cpp/include   -DROS_PACKAGE_NAME='"feature_manager"'

CXX_DEFINES = 

# TARGET_FLAGS = -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread

