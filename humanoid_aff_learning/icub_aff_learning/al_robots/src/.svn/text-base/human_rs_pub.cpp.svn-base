/*
 * human_rs_pub.cpp
 * Copyright (c) 2011, Kadir Firat Uyanik, Kovan Research Lab, METU
 * kadir@ceng.metu.edu.tr
 *
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>

std::map<std::string, double> joint_positions;
robot_state_publisher::RobotStatePublisher* pub_human_state;
ros::Subscriber sub_js;
KDL::Tree* tree_human;
ros::NodeHandle* nh;
std::map<std::string, double> map_joints;
bool first_js_msg_rcvd = false;

void jsRcvdCallback(sensor_msgs::JointStateConstPtr msg_js);

void init()
{
  nh = new ros::NodeHandle();
  tree_human = new KDL::Tree ();

  std::string robot_desc_string;
  nh->param ("robot_description", robot_desc_string, std::string ());
  if (!kdl_parser::treeFromString (robot_desc_string, *tree_human))
  {
    ROS_ERROR("Failed to construct kdl tree, exiting!");
    exit(-1);
  }

  pub_human_state = new robot_state_publisher::RobotStatePublisher (*tree_human);
  sub_js = nh->subscribe<sensor_msgs::JointState>("human_joint_states", 10, &jsRcvdCallback);
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "human_rs_publisher");
  init();

  ros::Rate rate (50.0);
  init ();

  while(!first_js_msg_rcvd && nh->ok())
  {
    ros::spinOnce();
    rate.sleep ();
  }

  while (nh->ok ())
  {
    pub_human_state->publishTransforms(map_joints, ros::Time::now());
    ros::spinOnce();
    rate.sleep ();
  }

  return 0;
}

void jsRcvdCallback(sensor_msgs::JointStateConstPtr msg_js)
{
  if(!first_js_msg_rcvd) first_js_msg_rcvd=true;

  for(uint i=0;i<msg_js->name.size();i++)
  {
    map_joints[msg_js->name[i]]=msg_js->position[i];
//    std::cout<<msg_js->name[i]<<" "<<msg_js->position[i]<<std::endl;
  }
}
