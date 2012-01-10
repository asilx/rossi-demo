/*
 * human_js_pub.cpp
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
#include <tf/transform_listener.h>
#include "sensor_msgs/JointState.h"
#include "boost/assign.hpp"

#include "al_utils/al_utils.h"

using namespace al::naming;

std::map<int, std::string> map_axial_int_2_str;
sensor_msgs::JointState js;

ros::Publisher pub_js;
ros::NodeHandle* nh;
tf::TransformListener* listener;
std::map<std::string, double> map_joints;

void
init ();

void
update ();

bool
extractJointPositions (const std::string frame_parent, const std::string frame_child);

void
publishJointPositions ();

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "human_js_publisher");
  init ();
  ros::Rate r (50.0);
  while (nh->ok ())
  {
    update ();
    ros::spinOnce ();
    r.sleep ();
  }

  return 0;
}

void
init ()
{
  nh = new ros::NodeHandle ();
  pub_js = nh->advertise<sensor_msgs::JointState> ("human_joint_states", 10);
  listener = new tf::TransformListener ();

  boost::assign::insert (map_axial_int_2_str) (0, "x") (1, "y") (2, "z");
  bool available = false;
  while (!available)
  {
    available = listener->waitForTransform (OPENNI_DEPTH_FRAME.c_str (), createFrameName (TORSO).c_str (),
                                            ros::Time::now (), ros::Duration (0.1));
    ros::spinOnce ();
  }
}

void
update ()
{
  std::string frame_parent = OPENNI_DEPTH_FRAME;
  std::string frame_child = TORSO;
  bool tf_healthy = extractJointPositions (OPENNI_DEPTH_FRAME, createFrameName (frame_child));
  frame_parent = TORSO;
  frame_child = LEFT_SHOULDER;
  tf_healthy = extractJointPositions (createFrameName (frame_parent), createFrameName (frame_child));
  if (tf_healthy)
    publishJointPositions ();
  //	frame_child = RIGHT_SHOULDER;
  //	extractJointPositions(frame_parent, frame_child);
  //	frame_child = NECK;
  //	extractJointPositions(frame_parent, frame_child);
  //	frame_child = HEAD;
  //	extractJointPositions(frame_parent, frame_child);
}

bool
extractJointPositions (const std::string frame_parent, const std::string frame_child)
{
  tf::StampedTransform transform;
  try
  {
    listener->lookupTransform (frame_parent, frame_child, ros::Time (0), transform);
  }
  catch (tf::TransformException ex)
  {
    return false;
  }
  tf::Vector3 offset;
  offset = transform.getOrigin();
  std::string tmp_parent_frame;
  if(frame_parent.compare(OPENNI_DEPTH_FRAME))
    tmp_parent_frame = getPartName (frame_parent);
  else
    tmp_parent_frame = OPENNI_DEPTH_FRAME;
  std::string tmp_child_frame;
  for (uint i = 0; i < 3; i++)
  {
    tmp_child_frame = getPartName (frame_child)+ "_" + map_axial_int_2_str[i];
    std::string joint_name = tmp_parent_frame + "_" + tmp_child_frame;
    map_joints[joint_name] = offset.m_floats[i];
    tmp_parent_frame = tmp_child_frame;
  }
  std::string joint_name = tmp_parent_frame + "_" + getPartName (frame_child);
  map_joints[joint_name] = 0;//fixed joint
  return true;
}

void
publishJointPositions ()
{
  js.name.resize (map_joints.size ());
  js.position.resize (map_joints.size ());

  std::map<std::string, double>::iterator it;

  int cnt = 0;
  for (it = map_joints.begin (); it != map_joints.end (); ++it)
  {
    js.name[cnt] = (*it).first;
    js.position[cnt] = (*it).second;
    std::cout << js.name[cnt] << " " << js.position[cnt] << std::endl;
    js.header.stamp = ros::Time::now ();
    cnt++;
  }

  //  std::cout << js << std::endl;
  if (pub_js.getNumSubscribers ())
    pub_js.publish (js);
}
