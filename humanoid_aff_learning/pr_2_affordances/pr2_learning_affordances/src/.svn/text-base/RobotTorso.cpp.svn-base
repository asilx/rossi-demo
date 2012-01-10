/*
 * RobotTorso.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: kadir
 */

#include "../include/pr2_learning_affordances/RobotTorso.h"

RobotTorso::RobotTorso ()
{
  torso_client_ = new TorsoClient ("torso_controller/position_joint_action", true);

  //wait for the action server to come up
  while (!torso_client_->waitForServer (ros::Duration (5.0)))
  {
    ROS_INFO("Waiting for the torso action server to come up");
  }
}

RobotTorso::~RobotTorso ()
{
  delete torso_client_;
}

//tell the torso to go up
void
RobotTorso::up ()
{
  pr2_controllers_msgs::SingleJointPositionGoal up;
  up.position = 0.195; //all the way up is 0.2
  up.min_duration = ros::Duration (2.0);
  up.max_velocity = 1.0;

  ROS_INFO("Sending up goal");
  torso_client_->sendGoal (up);
  torso_client_->waitForResult ();
}

//tell the torso to go down
void
RobotTorso::down ()
{
  pr2_controllers_msgs::SingleJointPositionGoal down;
  down.position = 0.0;
  down.min_duration = ros::Duration (2.0);
  down.max_velocity = 1.0;

  ROS_INFO("Sending down goal");
  torso_client_->sendGoal (down);
  torso_client_->waitForResult ();
}
