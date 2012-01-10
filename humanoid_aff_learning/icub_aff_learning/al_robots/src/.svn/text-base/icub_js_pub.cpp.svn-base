/*
 * icub_js_pub.cpp
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
 *     * Neither the name of Kovan Lab nor the names of its
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

#include "iostream"
#include "al_robots/iCub.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

ros::Publisher pub_js;
ros::NodeHandle* nh;
std::map<std::string, double> map_js_;
sensor_msgs::JointState js;

double deg_rad(double deg)
{
  double rad = deg * 3.14/180;
  return rad;
}

void publishJointStates(std::map<std::string, double>& map_js)
{
  js.name.resize (map_js.size ());
  js.position.resize (map_js.size ());

  std::map<std::string, double>::iterator it;

  int cnt = 0;
  for (it = map_js.begin (); it != map_js.end (); ++it)
  {
    js.name[cnt] = (*it).first;
    js.position[cnt] = deg_rad((*it).second);
    std::cout << js.name[cnt] << " " << js.position[cnt] << std::endl;
    js.header.stamp = ros::Time::now ();
    cnt++;
  }

  //  std::cout << js << std::endl;
  if (pub_js.getNumSubscribers ())
    pub_js.publish (js);
}

int main(int argc, char** argv)
{

  ros::init (argc, argv, "icub_js_publisher");

  nh = new ros::NodeHandle ();
  pub_js = nh->advertise<sensor_msgs::JointState> ("icub_joint_states", 10);
  al::robot::iCub* icub = new al::robot::iCub("icub");

  icub->init ();
  ros::WallRate r (50.0);
  while (nh->ok ())
  {
    icub->getAllJointPos(map_js_);
    publishJointStates(map_js_);
    ros::spinOnce ();
    r.sleep ();
  }

  return 0;
}
