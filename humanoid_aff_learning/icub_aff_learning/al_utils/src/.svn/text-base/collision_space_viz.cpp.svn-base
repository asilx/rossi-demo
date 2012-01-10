/*
 * collision_space_viz.cpp
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

#include "arm_navigation_msgs/GetPlanningScene.h"
#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"
#include "al_utils/al_utils.h"

std::vector<arm_navigation_msgs::CollisionObject> collision_objects;
std::vector<arm_navigation_msgs::CollisionObject> prev_collision_objects;
visualization_msgs::MarkerArray bboxes;
visualization_msgs::MarkerArray prev_bboxes;
ros::Publisher pub_bboxes;

void
publishAllObjectBoxes (const std::vector<arm_navigation_msgs::CollisionObject>& objects);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "collision_space_viz");
  ros::NodeHandle nh;
  ros::ServiceClient srv_cl_get_planning_scene;

  arm_navigation_msgs::GetPlanningScene planning_scene;

  pub_bboxes = nh.advertise<visualization_msgs::MarkerArray> ("/collision_objects_viz", 5);

  srv_cl_get_planning_scene
      = nh.serviceClient<arm_navigation_msgs::GetPlanningScene> ("/environment_server/get_planning_scene", true);
  srv_cl_get_planning_scene.waitForExistence ();
  srv_cl_get_planning_scene
      = nh.serviceClient<arm_navigation_msgs::GetPlanningScene> ("/environment_server/get_planning_scene", true);

  ros::Rate r (5);
  while (nh.ok ())
  {
    if (srv_cl_get_planning_scene.call (planning_scene))
    {
      collision_objects = planning_scene.response.planning_scene.collision_objects;
      //      std::cout << collision_objects.size () << std::endl;
      publishAllObjectBoxes (collision_objects);
    }
    ros::spinOnce ();
    r.sleep ();
  }
  return 0;
}

void
publishAllObjectBoxes (const std::vector<arm_navigation_msgs::CollisionObject>& objects)
{
  //no one interested in bbox, just quit
  if (pub_bboxes.getNumSubscribers () == 0)
    return;

  std::cout<<objects.size()<<std::endl;
  bboxes.markers.resize (objects.size ());

  std::vector<bool> used_prev_collision_objects (prev_collision_objects.size (), false);
  for (uint8_t i = 0; i < objects.size (); i++)
  {
    for (uint8_t j = 0; j < prev_collision_objects.size (); j++)
    {
      if (objects[i].id == prev_collision_objects[j].id)
      {
        used_prev_collision_objects[j] = true;
        break;
      }
    }

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now ();
    marker.header.frame_id = objects[i].header.frame_id;
    marker.ns = "collision_boxes";
    marker.action = visualization_msgs::Marker::ADD;
    //    marker.header = objects[i].header;
    //    marker.type = objects[i].shapes[0].type;
    marker.type = arm_navigation_msgs::Shape::BOX;
    marker.lifetime = ros::Duration(1.0);
    marker.pose = objects[i].poses[0];
    marker.scale.x = objects[i].shapes[0].dimensions[0];
    marker.scale.y = objects[i].shapes[0].dimensions[1];
    marker.scale.z = objects[i].shapes[0].dimensions[2];
    marker.id = (int)i;
    marker.color = al::viz::colorize (marker.id, 0.5);
    bboxes.markers[i]=marker;
  }

  visualization_msgs::MarkerArray tmp_bboxes = bboxes;

  for(uint i=0;i<prev_collision_objects.size();i++)
  {
    if(!used_prev_collision_objects[i])
    {
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now ();
      marker.header.frame_id = prev_collision_objects[i].header.frame_id;
      marker.ns = "collision_boxes";
      marker.action = visualization_msgs::Marker::DELETE;
      marker.id = prev_bboxes.markers[i].id;
      bboxes.markers.push_back(marker);
    }
  }

  prev_bboxes = tmp_bboxes;

  //now publish bboxes for every valid object
//  for (uint8_t i = 0; i < objects.size (); i++)
//  {
//    //    std::cout << objects[i] << std::endl;
//
//    visualization_msgs::Marker marker;
//    marker.header.stamp = ros::Time::now ();
//    marker.header.frame_id = objects[i].header.frame_id;
//    marker.ns = "collision_boxes";
//    marker.action = visualization_msgs::Marker::ADD;
//
//    //    marker.header = objects[i].header;
//    //    marker.type = objects[i].shapes[0].type;
//    marker.type = arm_navigation_msgs::Shape::BOX;
//
//    marker.pose = objects[i].poses[0];
//    marker.scale.x = objects[i].shapes[0].dimensions[0];
//    marker.scale.y = objects[i].shapes[0].dimensions[1];
//    marker.scale.z = objects[i].shapes[0].dimensions[2];
//    marker.id = (int)i;
//    marker.color = al::viz::colorize (marker.id, 0.5);
//    //    marker.lifetime = ros::Duration();
//
//    //    std::string object_id = objects[i].id;
//    //    if (object_id.find ("table") != object_id.npos)
//    //    {
//    //      marker.color.a = 1;
//    //      marker.color.r = 0.5;
//    //      marker.color.g = 0.5;
//    //      marker.color.b = 0.5;
//    ////      marker.id = 10;
//    //    }
//    //    else
//    //    {
//    //      std::stringstream s(objects[i].id);
//    //      int marker_id =0;
//    //      s >> marker_id;
//    ////      marker.id = marker_id;
//    //      marker.color = al::viz::colorize (marker_id, 0.5);
//    //    }
//
//    bboxes.markers.push_back (marker);
//  }
  ROS_DEBUG("*** Publishing Bounding Boxes! ***");
  if (pub_bboxes.getNumSubscribers ())
    pub_bboxes.publish (bboxes);
}
