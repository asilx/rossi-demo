#include "gazebo_plugins/gazebo_ros_factory.h"
#include "gazebo/XMLConfig.hh"
#include "gazebo/SpawnModel.h"

#include "ros/package.h"
#include "ros/ros.h"

#include "iostream"
#include "sstream"
#include "string"

#include "affordances_msgs/PerceiveRangeData.h"
#include "pr2_learning_affordances/ExecuteCartesianIKTrajectory.h"

//this can be used somewhere else:
//!ros::service::waitForService("/execute_cartesian_ik_trajectory",ros::Duration(2.0)

ros::Publisher pub_aff;

void requestRangeData()
{
  affordances_msgs::PerceiveRangeData aff_msg;
  aff_msg.do_perceive = true;
  pub_aff.publish(aff_msg);
}

bool reachHome()
{
  pr2_learning_affordances::ExecuteCartesianIKTrajectory exec_cart_traj;
  exec_cart_traj.request.header.frame_id = "/base_link";
  geometry_msgs::Pose pose;

  pose.position.x = 0.55;
  pose.position.y = -0.6;
  pose.position.z = 0.60;

  pose.orientation.w = 1;
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;

  exec_cart_traj.request.poses.push_back(pose);
  //waits here, until service is finished
  bool service_called = ros::service::call("/execute_cartesian_ik_trajectory", exec_cart_traj);
  if(service_called!=1)
    return false;
  ros::spinOnce();
  std::cout<<"home finished"<<std::endl;
  return true;
}

bool pushObject()
{
  pr2_learning_affordances::ExecuteCartesianIKTrajectory exec_cart_traj;
  exec_cart_traj.request.header.frame_id = "/base_link";
  geometry_msgs::Pose pose;

  pose.position.x = 0.55;
  pose.position.y = -0.05;
  pose.position.z = 0.60;

  pose.orientation.w = 1;
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;

  exec_cart_traj.request.poses.push_back(pose);
  bool service_called = ros::service::call("/execute_cartesian_ik_trajectory", exec_cart_traj);
  if(service_called!=1)
    return false;
  ros::spinOnce();
  std::cout<<"push finished"<<std::endl;
  return true;
}

bool reachObject()
{
  pr2_learning_affordances::ExecuteCartesianIKTrajectory exec_cart_traj;
  exec_cart_traj.request.header.frame_id = "/base_link";
  geometry_msgs::Pose pose;

  pose.position.x = 0.55;
  pose.position.y = -0.45;
  pose.position.z = 0.60;

  pose.orientation.w = 1;
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;

  exec_cart_traj.request.poses.push_back(pose);
  bool service_called = ros::service::call("/execute_cartesian_ik_trajectory", exec_cart_traj);
  if(service_called!=1)
    return false;
  ros::spinOnce();
  std::cout<<"reach finished"<<std::endl;
  return true;
}

bool addObject(std::string object_model_file_name)
{
  gazebo::SpawnModel spawn_model;
  std::string file_path = ros::package::getPath("pr2_affordances_manager") + "/objects/" + object_model_file_name;
//  std::cout<<file_path<<std::endl;
  std::string file_contents;
  std::ifstream point_file(file_path.c_str());
  std::string line;
  if (point_file.is_open())
  {
//    std::cout<<"file is opened"<<std::endl;
    while (point_file.good())
    {
      getline(point_file, line);
      file_contents += line;
    }
    point_file.close();
  }
  spawn_model.request.model_name = "model";
  spawn_model.request.model_xml = file_contents;
  spawn_model.request.initial_pose.position.x =  0.65;
  spawn_model.request.initial_pose.position.y = -0.3;
  spawn_model.request.initial_pose.position.z =  0.9;
//  std::cout<<file_contents<<std::endl;
  bool service_called = ros::service::call("gazebo/spawn_gazebo_model", spawn_model);
  if (service_called)
  {
    if (spawn_model.response.success)
    {
      std::cout<<"service is called"<<std::endl;
      return true;
    }
    else
    {
      ROS_ERROR_STREAM(spawn_model.response.status_message);
      return false;
    }
  }
  else
  {
    std::cout<<"service is not called"<<std::endl;
    return false;
  }
}

int
main (int argc, char **argv)
{
  //  gazebo::Entity* table_model = gazebo::World::Instance()->GetEntityByName("table_model");
  //  gazebo::GazeboRosFactory* gazebo_env;
  //  gazebo_env = new gazebo::GazeboRosFactory(table_model->GetParent());
  //  gazebo::XMLConfigNode* node = new gazebo::XMLConfigNode()
  //  gazebo_env->Load()

  ros::init (argc, argv, "manager");
  ros::NodeHandle nh;
  pub_aff = nh.advertise<affordances_msgs::PerceiveRangeData>("/affordances/get_range_data",10);

  //Instructions:
  // 1- Add an object
  // 2- move robot arm next to the object
  // 3- get range data (point cloud)
  // 4- apply push action
  // 5- get range data after action is completed

  reachHome();
//  addObject("box.model");
  addObject("sphere.model");
  usleep(100000);
  reachObject();
  requestRangeData();

  pushObject();
  usleep(100000);
  requestRangeData();

  return 0;
}
