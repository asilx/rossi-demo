/*
 * al_utils.cpp
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

#include "al_utils/al_utils.h"

bool
al::behavior::getTransformBetween (const std::string parent_frame, const std::string child_frame, tf::StampedTransform& transform)
{
  static tf::TransformListener listener;

  bool transformation_healthy = true;
  try
  {
    listener.lookupTransform (parent_frame.c_str (), child_frame.c_str (), ros::Time (0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    transformation_healthy = false;
  }
  return transformation_healthy;
}

std_msgs::ColorRGBA
al::viz::colorize (int color_id, float alpha)
{
  std_msgs::ColorRGBA color;
  color.a = alpha;
  switch (color_id)
  {
    case RED:
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      break;
    case GREEN:
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
      break;
    case BLUE:
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
      break;
    case YELLOW:
      color.r = 1.0;
      color.g = 1.0;
      color.b = 0.0;
      break;
    case CYAN:
      color.r = 0.0;
      color.g = 1.0;
      color.b = 1.0;
      break;
    case MAGENTA:
      color.r = 1.0;
      color.g = 0.0;
      color.b = 1.0;
      break;
    case BLACK:
      color.r = 0.0;
      color.g = 0.0;
      color.b = 0.0;
      break;
    case WHITE:
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;
      break;
    default:
      color.r = 0.5;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }
  return color;
}

std::string
al::system::execCmd (std::string sys_cmd)
{
  FILE* pipe = popen (sys_cmd.c_str (), "r");
  if (!pipe)
    return "ERROR";
  char buffer[128];
  std::string result = "";
  while (!feof (pipe))
  {
    if (fgets (buffer, 128, pipe) != NULL)
      result += buffer;
  }
  pclose (pipe);
  return result;
}

int
al::system::execSysCmd (std::string sys_cmd)
{
  return std::system (sys_cmd.c_str ());
}

int
al::system::getProcessIdByName (std::string p_name)
{
  int pid = 0;
  //if tabletop_segmentation service is already running, don't re-roslaunch
  std::string str_pid = execCmd ("pidof " + p_name);
  std::stringstream ss (str_pid);
  ss >> pid;

  return pid;
}

int
al::system::killProcessByName (std::string p_name, int kill_signal)
{
  return kill (getProcessIdByName (p_name), kill_signal);
}

int
al::system::killNodeByName (std::string n_name)
{
  return execSysCmd (("rosnode kill " + n_name).c_str ());
}

float
al::math::getDistanceBetween (const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2)
{
  float d_x = v1.x - v2.x;
  float d_y = v1.y - v2.y;
  float d_z = v1.z - v2.z;

  return sqrt (d_x * d_x + d_y * d_y + d_z * d_z);
}

float
al::math::getDistanceBetween (const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  float d_x = p1.x - p2.x;
  float d_y = p1.y - p2.y;
  float d_z = p1.z - p2.z;

  return sqrt (d_x * d_x + d_y * d_y + d_z * d_z);
}

al::math::BoundingBox
al::math::calCloudBBox (const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_data)
{
  BoundingBox bbox;
  if (!cluster_data->points.size ())
  {
    bbox.bbox_valid_ = false;
  }
  else
  {
    //        float x_min = FLT_MAX;
    //        float y_min = FLT_MAX;
    //        float z_min = FLT_MAX;
    //        float x_max = FLT_MIN;
    //        float y_max = FLT_MIN;
    //        float z_max = FLT_MIN;

    float x_min = 100;
    float y_min = 100;
    float z_min = 100;
    float x_max = -100;
    float y_max = -100;
    float z_max = -100;

    for (uint i = 0; i < cluster_data->points.size (); i++)
    {
      if (isnan (cluster_data->points[i].x) || isnan (cluster_data->points[i].y) || isnan (cluster_data->points[i].z))
        continue;

      x_max = std::max (x_max, cluster_data->points[i].x);
      y_max = std::max (y_max, cluster_data->points[i].y);
      z_max = std::max (z_max, cluster_data->points[i].z);

      x_min = std::min (x_min, cluster_data->points[i].x);
      y_min = std::min (y_min, cluster_data->points[i].y);
      z_min = std::min (z_min, cluster_data->points[i].z);
    }

    std::cout << x_min << " " << y_min << " " << z_min << " " << x_max << " " << y_max << " " << z_max << std::endl;

    if (x_min == 100 || y_min == 100 || z_min == 100 || x_max == -100 || y_max == -100 || z_max == -100)
      bbox.bbox_valid_ = false;
    else
    {
      bbox.bbox_pose_.position.x = (x_min + x_max) / 2;
      bbox.bbox_pose_.position.y = (y_min + y_max) / 2;
      bbox.bbox_pose_.position.z = (z_min + z_max) / 2;

      bbox.bbox_dims_.x = x_max - x_min;
      bbox.bbox_dims_.y = y_max - y_min;
      bbox.bbox_dims_.z = z_max - z_min;

      bbox.bbox_pose_.orientation.x = 0.0;
      bbox.bbox_pose_.orientation.y = 0.0;
      bbox.bbox_pose_.orientation.z = 0.0;
      bbox.bbox_pose_.orientation.w = 1.0;
      bbox.bbox_valid_ = true;
    }
  }
  return bbox;
}

std::string
al::naming::createFrameName (std::string part, int user_id)
{
  std::stringstream s;
  s << user_id;
  return part + "_" + s.str ();
}

std::string
al::naming::createJointName (std::string part_parent, std::string part_child, int dimension)
{
  std::string s_dimension;
  std::stringstream s;
  s << dimension;
  return part_parent + "_" + part_child + "_" + s.str ();
}

std::string
al::naming::createJointName (std::string part_parent, std::string part_child, std::string dim)
{
  return part_parent + "_" + part_child + "_" + dim;
}

std::string
al::naming::getPartName (std::string frame_name)
{
  size_t pos = frame_name.find_last_of ("_");
  return frame_name.substr (0, pos);
}
