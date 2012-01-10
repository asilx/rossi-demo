#include "pr2_learning_affordances/RobotArm.h"
#include "pr2_learning_affordances/RobotTorso.h"

int
main (int argc, char** argv)
{
  // Init the ROS node
  ros::init (argc, argv, "robot_driver");

  RobotTorso torso;
  RobotArm arm_l (LEFT);
  RobotArm arm_r (RIGHT);
  // Start the trajectory
  torso.up ();
  arm_l.initArmControl (JOINT);
  arm_r.initArmControl (JOINT);
  arm_l.startTrajectory (arm_l.oufOfTableTrajectory ());
  arm_r.startTrajectory (arm_r.oufOfTableTrajectory ());

  // Wait for trajectory completion
  while (!arm_l.getState ().isDone () && !arm_r.getState ().isDone () && ros::ok ())
  {
    usleep (50000);
  }

  arm_r.initArmControl (CARTESIAN);
//  double start_angles[7];
//  arm_r.get_current_joint_angles(start_angles);
//  double target_angles[7];
//  geometry_msgs::PoseStamped pose;
//  pose.pose.position.x = 0.76;
//  pose.pose.position.y = -0.19;
//  pose.pose.position.z = 0.83;
//  pose.pose.orientation.w = 0.02;
//  pose.pose.orientation.x = -0.09;
//  pose.pose.orientation.y = 0.0;
//  pose.pose.orientation.z = 1.0;
//
//  pose.header.frame_id = "/base_link";
  ROS_INFO("Waiting for cartesian trajectories to execute");
  ros::spin ();

  return 0;
}
