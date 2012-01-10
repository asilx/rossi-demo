/*
 * al_utils.h
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

#ifndef AL_UTILS_H_
#define AL_UTILS_H_

#include "iostream"
#include "string"
#include "sstream"
#include "vector"
#include "limits"

#include "stdio.h"
#include "signal.h"

#include "tf/transform_listener.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"

#include "opencv2/opencv.hpp"

#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

namespace al
{
  namespace behavior
  {
    bool
    getTransformBetween (const std::string parent_frame, const std::string child_frame, tf::StampedTransform& transform);
  }

  namespace viz
  {
    enum ColorID
    {
      RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, BLACK, WHITE
    };

    std_msgs::ColorRGBA
    colorize (int color_id, float alpha);
  }

  namespace system
  {
    std::string
    execCmd (std::string sys_cmd);

    int
    execSysCmd (std::string sys_cmd);

    int
    getProcessIdByName (std::string p_name);

    int
    killProcessByName (std::string p_name, int kill_signal = SIGINT);

    int
    killNodeByName (std::string n_name);
  }

  namespace math
  {
    struct BoundingBox
    {
      geometry_msgs::Pose bbox_pose_;
      geometry_msgs::Vector3 bbox_dims_;
      bool bbox_valid_;

      BoundingBox ()
      {
        bbox_valid_ = false;
      }

      BoundingBox (geometry_msgs::Pose bbox_pose, geometry_msgs::Vector3 bbox_dims)
      {
        bbox_pose_ = bbox_pose;
        bbox_dims_ = bbox_dims;
        if (bbox_dims_.x <= 0 || bbox_dims_.y <= 0 || bbox_dims_.z <= 0)
        {
          bbox_valid_ = false;
        }
        else
          bbox_valid_ = true;
      }
    };

    const float PI = 3.14159265;
    const float PI_2 = 1.57079633;

    float
    getDistanceBetween (const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2);

    float
    getDistanceBetween (const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);

    BoundingBox
    calCloudBBox (const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_data);
  }

  namespace naming
  {
    const std::string OPENNI_DEPTH_FRAME = "openni_depth_frame";

    const std::string LEFT_SHOULDER = "left_shoulder";
    const std::string RIGHT_SHOULDER = "right_shoulder";

    const std::string LEFT_ELBOW = "left_elbow";
    const std::string RIGHT_ELBOW = "right_elbow";

    const std::string LEFT_HAND = "left_hand";
    const std::string RIGHT_HAND = "right_hand";

    const std::string TORSO = "torso";
    const std::string NECK = "neck";
    const std::string HEAD = "head";

    std::string
    createFrameName (std::string part, int user_id = 1);

    std::string
    createJointName (std::string part_parent, std::string part_child, int dimension);

    std::string
    createJointName (std::string part_parent, std::string part_child, std::string dim);

    std::string
    getPartName (std::string frame_name);
  }
}
#endif /* AL_UTILS_H_ */
