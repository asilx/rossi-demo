#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm_msgs/MoveArmAction.h>

enum WhichHand
{
  LEFT = -1, RIGHT = 1
};

void
moveTorso ();

void
moveArm (const ros::NodeHandle &nh, int which_hand = -1);

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;

  moveArm (nh, RIGHT);
  moveArm (nh, LEFT);

  ros::shutdown ();
}

void
moveArm (const ros::NodeHandle &nh, int which_hand)
{
  std::string name("move_arm");
  if(which_hand == LEFT)
  {
    name.insert(5, "left_");
    std::cout<<name<<std::endl;
  }
  else
  {
    name.insert(5, "right_");
    std::cout<<name<<std::endl;
  }
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm (name, true);

  ROS_INFO("Waiting for connecting to server");
  move_arm.waitForServer ();
  ROS_INFO("Connected to server");

  move_arm_msgs::MoveArmGoal goalB;
  std::vector<std::string> names (7);
  std::string shoulder_pan = "_shoulder_pan_joint";
  std::string shoulder_lift = "_shoulder_lift_joint";
  std::string upper_arm_roll = "_upper_arm_roll_joint";
  std::string elbow_flex = "_elbow_flex_joint";
  std::string forearm_roll = "_forearm_roll_joint";
  std::string wrist_flex = "_wrist_flex_joint";
  std::string wrist_roll = "_wrist_roll_joint";

  if (which_hand == LEFT)
  {
    names[0] = "l" + shoulder_pan;
    names[1] = "l" + shoulder_lift;
    names[2] = "l" + upper_arm_roll;
    names[3] = "l" + elbow_flex;
    names[4] = "l" + forearm_roll;
    names[5] = "l" + wrist_flex;
    names[6] = "l" + wrist_roll;

    goalB.motion_plan_request.group_name = "left_arm";
  }
  else
  {
    names[0] = "r" + shoulder_pan;
    names[1] = "r" + shoulder_lift;
    names[2] = "r" + upper_arm_roll;
    names[3] = "r" + elbow_flex;
    names[4] = "r" + forearm_roll;
    names[5] = "r" + wrist_flex;
    names[6] = "r" + wrist_roll;

    goalB.motion_plan_request.group_name = "right_arm";
  }

  goalB.motion_plan_request.num_planning_attempts = 1;
  goalB.motion_plan_request.allowed_planning_time = ros::Duration (5.0);

  goalB.motion_plan_request.planner_id = std::string ("");
  goalB.planner_service_name = std::string ("ompl_planning/plan_kinematic_path");
  goalB.motion_plan_request.goal_constraints.joint_constraints.resize (names.size ());

  for (unsigned int i = 0; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size (); ++i)
  {
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }

  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -2.135 * which_hand;
  goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = 0.803;
  goalB.motion_plan_request.goal_constraints.joint_constraints[2].position = -1.732 * which_hand;
  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = -1.905;
  goalB.motion_plan_request.goal_constraints.joint_constraints[4].position = -2.369 * which_hand;
  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = -1.680;
  goalB.motion_plan_request.goal_constraints.joint_constraints[6].position = 1.398;

  if (nh.ok ())
  {
    bool finished_within_time = false;
    move_arm.sendGoal (goalB);
    finished_within_time = move_arm.waitForResult (ros::Duration (200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal ();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState ();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if (success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
}
