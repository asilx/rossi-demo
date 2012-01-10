/*
 * RobotArm.h
 *
 *  Created on: Aug 17, 2011
 *      Author: kadir
 */

#ifndef ROBOTARM_H_
#define ROBOTARM_H_

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include "pr2_learning_affordances/ExecuteCartesianIKTrajectory.h"

#define MAX_JOINT_VEL 0.5  //in radians/sec

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;
typedef actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> MoveArmClient;

static const std::string RIGHT_ARM_IK_NAME = "/pr2_right_arm_kinematics/get_ik";
static const std::string LEFT_ARM_IK_NAME = "/pr2_left_arm_kinematics/get_ik";

enum ArmType
{
  LEFT = -1, RIGHT = 1
};

enum ControlType
{
  JOINT, CARTESIAN
};

class RobotArm
{
private:
  // Action client for the joint trajectory action
  // used to trigger the arm movement action
  TrajClient* traj_client_;
  int arm_type_;

  ros::NodeHandle node;
  ros::ServiceClient ik_client;
  ros::ServiceServer service;
  kinematics_msgs::GetPositionIK::Request ik_request;
  kinematics_msgs::GetPositionIK::Response ik_response;
  pr2_controllers_msgs::JointTrajectoryGoal goal;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm (int arm_type = RIGHT);

  bool
  initArmControl (int control_type = CARTESIAN);

  bool
  initArmJointControl ();

  bool
  initArmCartControl ();

  //! Clean up the action client
  ~RobotArm ();

  //! Sends the command to start a given trajectory
  void
  startTrajectory (pr2_controllers_msgs::JointTrajectoryGoal goal);

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
   as a single trajectory. Alternatively, each of these waypoints could
   be in its own trajectory - a trajectory can have one or more waypoints
   depending on the desired application.
   */
  pr2_controllers_msgs::JointTrajectoryGoal
  oufOfTableTrajectory ();

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState
  getState ();

  //run inverse kinematics on a PoseStamped (7-dof pose
  //(position + quaternion orientation) + header specifying the
  //frame of the pose)
  //tries to stay close to double start_angles[7]
  //returns the solution angles in double solution[7]
  bool
  run_ik (geometry_msgs::PoseStamped pose, double start_angles[7], double solution[7], std::string link_name);

  //figure out where the arm is now
  void
  get_current_joint_angles (double current_angles[7]);

  //send a desired joint trajectory to the joint trajectory action
  //and wait for it to finish
  bool
  execute_joint_trajectory (std::vector<double *> joint_trajectory);

  //service function for execute_cartesian_ik_trajectory
  bool
  execute_cartesian_ik_trajectory (pr2_learning_affordances::ExecuteCartesianIKTrajectory::Request &req,
                                   pr2_learning_affordances::ExecuteCartesianIKTrajectory::Response &res);
};
#endif /* ROBOTARM_H_ */
