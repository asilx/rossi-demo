/*
 * RobotArm.cpp
 *
 *  Created on: Aug 17, 2011
 *      Author: kadir
 */

#include "pr2_learning_affordances/RobotArm.h"

RobotArm::RobotArm (int arm_type)
{
  arm_type_ = arm_type;
  // tell the action client that we want to spin a thread by default

  if (arm_type_ == LEFT)
    traj_client_ = new TrajClient ("l_arm_controller/joint_trajectory_action", true);
  else
    traj_client_ = new TrajClient ("r_arm_controller/joint_trajectory_action", true);

  // First, the joint names, which apply to all waypoints
  if (arm_type_ == LEFT)
  {
    goal.trajectory.joint_names.push_back ("l_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back ("l_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back ("l_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back ("l_elbow_flex_joint");
    goal.trajectory.joint_names.push_back ("l_forearm_roll_joint");
    goal.trajectory.joint_names.push_back ("l_wrist_flex_joint");
    goal.trajectory.joint_names.push_back ("l_wrist_roll_joint");
  }
  else
  {
    goal.trajectory.joint_names.push_back ("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back ("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back ("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back ("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back ("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back ("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back ("r_wrist_roll_joint");
  }
}

bool
RobotArm::initArmJointControl ()
{
  return true;
}

bool
RobotArm::initArmCartControl ()
{
  ik_client = node.serviceClient<kinematics_msgs::GetPositionIK> (RIGHT_ARM_IK_NAME, true);
  ROS_INFO("Waiting for services to be ready");
  if(arm_type_ == LEFT)
    ros::service::waitForService (LEFT_ARM_IK_NAME);
  else
    ros::service::waitForService (RIGHT_ARM_IK_NAME);
  ROS_INFO("Services ready");

  //register a service to input desired Cartesian trajectories
  service = node.advertiseService ("execute_cartesian_ik_trajectory", &RobotArm::execute_cartesian_ik_trajectory, this);

  return true;
}

bool
RobotArm::initArmControl (int control_type)
{
  while (ros::ok() && !traj_client_->waitForServer (ros::Duration (5.0)))
    ROS_INFO("Waiting for the joint_trajectory_action server");

  // wait for action server to come up
  if (control_type == JOINT)
    initArmJointControl();
  else
    initArmCartControl();

  return true;
}

RobotArm::~RobotArm ()
{
  delete traj_client_;
}

void
RobotArm::startTrajectory (pr2_controllers_msgs::JointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now () + ros::Duration (1.0);
  traj_client_->sendGoal (goal);
}

pr2_controllers_msgs::JointTrajectoryGoal
RobotArm::oufOfTableTrajectory ()
{
  // We will have two waypoints in this goal trajectory
  goal.trajectory.points.resize (2);

  // First trajectory point
  // Positions
  int ind = 0;
  goal.trajectory.points[ind].positions.resize (7);
  goal.trajectory.points[ind].positions[0] = 0.0;
  goal.trajectory.points[ind].positions[1] = 0.0;
  goal.trajectory.points[ind].positions[2] = 0.0;
  goal.trajectory.points[ind].positions[3] = 0.0;
  goal.trajectory.points[ind].positions[4] = 0.0;
  goal.trajectory.points[ind].positions[5] = 0.0;
  goal.trajectory.points[ind].positions[6] = 0.0;
  // Velocities
  goal.trajectory.points[ind].velocities.resize (7);
  for (size_t j = 0; j < 7; ++j)
  {
    goal.trajectory.points[ind].velocities[j] = 0.0;
  }
  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[ind].time_from_start = ros::Duration (1.0);

  // Second trajectory point
  // Positions
  ind += 1;
  goal.trajectory.points[ind].positions.resize (7);
  goal.trajectory.points[ind].positions[0] = -2.135 * arm_type_;
  goal.trajectory.points[ind].positions[1] = 0.803;
  goal.trajectory.points[ind].positions[2] = -1.732 * arm_type_;
  goal.trajectory.points[ind].positions[3] = -1.905;
  goal.trajectory.points[ind].positions[4] = -2.369 * arm_type_;
  goal.trajectory.points[ind].positions[5] = -1.680;
  goal.trajectory.points[ind].positions[6] = 1.398;

  // Velocities
  goal.trajectory.points[ind].velocities.resize (7);
  for (size_t j = 0; j < 7; ++j)
  {
    goal.trajectory.points[ind].velocities[j] = 0.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[ind].time_from_start = ros::Duration (2.0);

  //we are done; return the goal
  return goal;
}

actionlib::SimpleClientGoalState
RobotArm::getState ()
{
  return traj_client_->getState ();
}

bool
RobotArm::run_ik (geometry_msgs::PoseStamped pose, double start_angles[7], double solution[7], std::string link_name)
{
  kinematics_msgs::GetPositionIK::Request ik_request;
  kinematics_msgs::GetPositionIK::Response ik_response;

  ik_request.timeout = ros::Duration (5.0);
  if (arm_type_ == LEFT)
  {
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("l_shoulder_pan_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("l_shoulder_lift_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("l_upper_arm_roll_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("l_elbow_flex_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("l_forearm_roll_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("l_wrist_flex_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("l_wrist_roll_joint");
  }
  else
  {
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("r_shoulder_pan_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("r_shoulder_lift_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("r_upper_arm_roll_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("r_elbow_flex_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("r_forearm_roll_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("r_wrist_flex_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back ("r_wrist_roll_joint");
  }

  ik_request.ik_request.ik_link_name = link_name;

  ik_request.ik_request.pose_stamped = pose;
  ik_request.ik_request.ik_seed_state.joint_state.position.resize (7);

  for (int i = 0; i < 7; i++)
    ik_request.ik_request.ik_seed_state.joint_state.position[i] = start_angles[i];

  ROS_INFO("request pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

  bool ik_service_call = ik_client.call (ik_request, ik_response);
  if (!ik_service_call)
  {
    ROS_ERROR("IK service call failed!");
    return 0;
  }
  if (ik_response.error_code.val == ik_response.error_code.SUCCESS)
  {
    for (int i = 0; i < 7; i++)
    {
      solution[i] = ik_response.solution.joint_state.position[i];
    }
    ROS_INFO("solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f",
        solution[0],solution[1], solution[2],solution[3],
        solution[4],solution[5],solution[6]);
    ROS_INFO("IK service call succeeded");
    return 1;
  }
  ROS_INFO("IK service call error code: %d", ik_response.error_code.val);
  return 0;
}

//figure out where the arm is now
void
RobotArm::get_current_joint_angles (double current_angles[7])
{
  int i;

  //get a single message from the topic 'r_arm_controller/state'
  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg = ros::topic::waitForMessage<
      pr2_controllers_msgs::JointTrajectoryControllerState> ("r_arm_controller/state");

  //extract the joint angles from it
  for (i = 0; i < 7; i++)
  {
    current_angles[i] = state_msg->actual.positions[i];
  }
}

//send a desired joint trajectory to the joint trajectory action
//and wait for it to finish
bool
RobotArm::execute_joint_trajectory (std::vector<double *> joint_trajectory)
{
  int i, j;
  int trajectorylength = joint_trajectory.size ();

  //get the current joint angles
  double current_angles[7];
  get_current_joint_angles (current_angles);

  //fill the goal message with the desired joint trajectory
  goal.trajectory.points.resize (trajectorylength + 1);

  //set the first trajectory point to the current position
  goal.trajectory.points[0].positions.resize (7);
  goal.trajectory.points[0].velocities.resize (7);
  for (j = 0; j < 7; j++)
  {
    goal.trajectory.points[0].positions[j] = current_angles[j];
    goal.trajectory.points[0].velocities[j] = 0.0;
  }

  //make the first trajectory point start 0.25 seconds from when we run
  goal.trajectory.points[0].time_from_start = ros::Duration (0.25);

  //fill in the rest of the trajectory
  double time_from_start = 0.25;
  for (i = 0; i < trajectorylength; i++)
  {
    goal.trajectory.points[i + 1].positions.resize (7);
    goal.trajectory.points[i + 1].velocities.resize (7);

    //fill in the joint positions (velocities of 0 mean that the arm
    //will try to stop briefly at each waypoint)
    for (j = 0; j < 7; j++)
    {
      goal.trajectory.points[i + 1].positions[j] = joint_trajectory[i][j];
      goal.trajectory.points[i + 1].velocities[j] = 0.0;
    }

    //compute a desired time for this trajectory point using a max
    //joint velocity
    double max_joint_move = 0;
    for (j = 0; j < 7; j++)
    {
      double joint_move = fabs (goal.trajectory.points[i + 1].positions[j] - goal.trajectory.points[i].positions[j]);
      if (joint_move > max_joint_move)
        max_joint_move = joint_move;
    }
    double seconds = max_joint_move / MAX_JOINT_VEL;
    ROS_INFO("max_joint_move: %0.3f, seconds: %0.3f", max_joint_move, seconds);
    time_from_start += seconds;
    goal.trajectory.points[i + 1].time_from_start = ros::Duration (time_from_start);
  }

  //when to start the trajectory
  goal.trajectory.header.stamp = ros::Time::now () + ros::Duration (0.25);

  ROS_INFO("Sending goal to joint_trajectory_action");
  traj_client_->sendGoal (goal);

  traj_client_->waitForResult ();

  //get the current joint angles for debugging
  get_current_joint_angles (current_angles);
  ROS_INFO("joint angles after trajectory: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n",current_angles[0],current_angles[1],current_angles[2],current_angles[3],current_angles[4],current_angles[5],current_angles[6]);

  if (traj_client_->getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the arm finished the trajectory!");
    return 1;
  }
  ROS_INFO("The arm failed to execute the trajectory.");
  return 0;
}

//service function for execute_cartesian_ik_trajectory
bool
RobotArm::execute_cartesian_ik_trajectory (pr2_learning_affordances::ExecuteCartesianIKTrajectory::Request &req,
                                           pr2_learning_affordances::ExecuteCartesianIKTrajectory::Response &res)
{

  int trajectory_length = req.poses.size ();
  int i, j;

  //IK takes in Cartesian poses stamped with the frame they belong to
  geometry_msgs::PoseStamped stamped_pose;
  stamped_pose.header = req.header;
  stamped_pose.header.stamp = ros::Time::now ();
  bool success;
  std::vector<double *> joint_trajectory;

  //get the current joint angles (to find ik solutions close to)
  double last_angles[7];
  get_current_joint_angles (last_angles);

  //find IK solutions for each point along the trajectory
  //and stick them into joint_trajectory
  for (i = 0; i < trajectory_length; i++)
  {

    stamped_pose.pose = req.poses[i];
    double *trajectory_point = new double[7];
    success = run_ik (stamped_pose, last_angles, trajectory_point, "r_wrist_roll_link");
    joint_trajectory.push_back (trajectory_point);

    if (!success)
    {
      ROS_ERROR("IK solution not found for trajectory point number %d!\n", i);
      return 0;
    }
    for (j = 0; j < 7; j++)
      last_angles[j] = trajectory_point[j];
  }

  //run the resulting joint trajectory
  ROS_INFO("executing joint trajectory");
  success = execute_joint_trajectory (joint_trajectory);
  res.success = success;

  return success;
}
