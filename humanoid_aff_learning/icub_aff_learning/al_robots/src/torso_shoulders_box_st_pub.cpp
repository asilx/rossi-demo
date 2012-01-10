#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>

const std::string OPENNI_DEPTH_FRAME= "/openni_depth_frame";

const std::string LEFT_SHOULDER_FRAME = "/left_shoulder_1";
const std::string RIGHT_SHOULDER_FRAME = "/right_shoulder_1";
const std::string LEFT_ELBOW_FRAME = "/left_elbow_1";
const std::string RIGHT_ELBOW_FRAME = "/right_elbow_1";
const std::string LEFT_HAND_FRAME = "/left_hand_1";
const std::string RIGHT_HAND_FRAME = "/right_hand_1";
const std::string LEFT_HIP_FRAME = "/left_hip_1";
const std::string RIGHT_HIP_FRAME = "/right_hip_1";
const std::string LEFT_KNEE_FRAME = "/left_knee_1";
const std::string RIGHT_KNEE_FRAME = "/right_knee_1";
const std::string LEFT_FOOT_FRAME = "/left_foot_1";
const std::string RIGHT_FOOT_FRAME = "/right_foot_1";
const std::string NECK_FRAME = "/neck_1";
const std::string TORSO_FRAME = "/torso_1";
const std::string HEAD_FRAME = "/head_1";

const std::string LEFT_UPPER_ARM = "left_upper_arm";
const std::string LEFT_LOWER_ARM = "left_lower_arm";
const std::string LEFT_SHOULDER = "left_shoulder";
const std::string LEFT_ELBOW = "left_elbow";
const std::string LEFT_HAND = "left_hand";

const std::string TORSO_L_SHOULDER_X = "torso_to_l_shoulder_x";
const std::string L_SHOULDER_X_L_SHOULDER_Y = "l_shoulder_x_shoulder_y";
const std::string L_SHOULDER_Y_L_SHOULDER_Z = "l_shoulder_y_l_shoulder_z";
//const std::string L_SHOULDER_1_L_UPPER_ARM = "l_shoulder_1_l_upper_arm";
const std::string L_SHOULDER_L_ELBOW_X = "l_shoulder_l_elbow_x";
const std::string L_ELBOW_X_L_ELBOW_Y = "l_elbow_x_l_elbow_y";
const std::string L_ELBOW_Y_L_ELBOW_Z = "l_elbow_y_l_elbow_z";

//std::map<std::string, bool> healthy_transformation;
std::vector<std::string> frame_names (15);
std::vector<std::string> joint_names (10);
tf::TransformListener* listener;
std::map<std::string, double> joint_positions;
robot_state_publisher::RobotStatePublisher* pub_human_state;
KDL::Tree* tree_human;
KDL::SegmentMap* seg_map;
tf::TransformBroadcaster* tf_broadcaster;

void
init (std::map<std::string, tf::StampedTransform>& frame_transforms);

void
update (std::map<std::string, tf::StampedTransform>& frame_transforms);

//assumes torso related frames are not ill-transformed
void
extractTorsoJointsPositions (std::map<std::string, tf::StampedTransform>& frame_transforms, double& roll,
                             double& pitch, double& yaw);

//void
//extractShoulderJointState (const std::map<std::string, tf::StampedTransform>& frame_transforms);

int
main (int argc, char** argv)
{

  ros::init (argc, argv, "human_state_publisher");
  ros::NodeHandle node;
  listener = new tf::TransformListener ();
  tree_human = new KDL::Tree ();
  tf_broadcaster = new tf::TransformBroadcaster ();

  std::string robot_desc_string;
  node.param ("robot_description", robot_desc_string, std::string ());
  if (!kdl_parser::treeFromString (robot_desc_string, *tree_human))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  pub_human_state = new robot_state_publisher::RobotStatePublisher (*tree_human);
  seg_map = new KDL::SegmentMap (tree_human->getSegments ());
  std::map<std::string, KDL::TreeElement>::iterator it;
  it = seg_map->begin ();
  //
  for (uint i = 0; i < seg_map->size (); i++)
    std::cout << it->first << std::endl;

  //  KDL::Rotation rot = it->second.segment.getFrameToTip().M;
  //  KDL::Rotation rot = seg_map->find(LEFT_UPPER_ARM)->second.segment.getFrameToTip().M;

  //  listener.waitForTransform()

  ros::Rate rate (50.0);
  std::map<std::string, tf::StampedTransform> frame_transforms;
  init (frame_transforms);

  while (node.ok ())
  {
    update (frame_transforms);
    rate.sleep ();
  }

  return 0;
}

void
init (std::map<std::string, tf::StampedTransform>& frame_transforms)
{
  frame_names[0] = LEFT_SHOULDER_FRAME;
  frame_names[1] = RIGHT_SHOULDER_FRAME;
  frame_names[2] = LEFT_ELBOW_FRAME;
  frame_names[3] = RIGHT_ELBOW_FRAME;
  frame_names[4] = LEFT_HAND_FRAME;
  frame_names[5] = RIGHT_HAND_FRAME;
  frame_names[6] = LEFT_HIP_FRAME;
  frame_names[7] = RIGHT_HIP_FRAME;
  frame_names[8] = LEFT_KNEE_FRAME;
  frame_names[9] = RIGHT_KNEE_FRAME;
  frame_names[10] = LEFT_FOOT_FRAME;
  frame_names[11] = RIGHT_FOOT_FRAME;
  frame_names[12] = NECK_FRAME;
  frame_names[13] = TORSO_FRAME;
  frame_names[14] = HEAD_FRAME;

  joint_names[0] = TORSO_L_SHOULDER_X;
  joint_names[1] = L_SHOULDER_X_L_SHOULDER_Y;
  joint_names[2] = L_SHOULDER_Y_L_SHOULDER_Z;
  joint_names[3] = L_SHOULDER_L_ELBOW_X;
  joint_names[4] = L_ELBOW_X_L_ELBOW_Y;
  joint_names[5] = L_ELBOW_Y_L_ELBOW_Z;

  update (frame_transforms);
}

void
update (std::map<std::string, tf::StampedTransform>& frame_transforms)
{
  tf::StampedTransform transform;
  bool transformation_healthy = true;
  for (uint i = 0; i < frame_names.size (); i++)
  {
    try
    {
      listener->lookupTransform (OPENNI_DEPTH_FRAME.c_str(), frame_names[i].c_str (), ros::Time (0), transform);
      frame_transforms[frame_names[i]] = transform;
      //      healthy_transformation[frame_names[i]] = true;
    }
    catch (tf::TransformException ex)
    {
      //      ROS_ERROR("%s",ex.what());
      //      healthy_transformation[frame_names[i]] = false;
      transformation_healthy = false;
    }
  }

  //Do sth for healthy test, first calibrate the torso size.
  //check later to see if the joints are found correctly
  //broadcast missing frames now

  KDL::Rotation rot = seg_map->find (LEFT_UPPER_ARM)->second.segment.getFrameToTip ().M;
  KDL::Vector off = seg_map->find (LEFT_UPPER_ARM)->second.segment.getFrameToTip ().p;

  tf::Vector3 offset (off.x (), off.y (), off.z ());
  double x, y, z, w;
  rot.GetQuaternion (x, y, z, w);

  tf::Quaternion q;
  q.setValue (x, y, z, w);

  transform.setOrigin (offset);
  transform.setRotation (q);
  transform.frame_id_ = LEFT_SHOULDER_FRAME;
  transform.child_frame_id_ = LEFT_UPPER_ARM;
  transform.stamp_ = ros::Time::now ();
  /*tf::StampedTransform (transform, ros::Time::now ())*/
  tf_broadcaster->sendTransform (transform);

//  rot = seg_map->find (LEFT_LOWER_ARM)->second.segment.getFrameToTip ().M;
//  off = seg_map->find (LEFT_LOWER_ARM)->second.segment.getFrameToTip ().p;
//
//  offset = tf::Vector3 (off.x (), off.y (), off.z ());
//  rot.GetQuaternion (x, y, z, w);
//  q.setValue (x, y, z, w);
//
//  transform.setOrigin (offset);
//  transform.setRotation (q);
//  transform.frame_id_ = LEFT_SHOULDER_FRAME;
//  transform.child_frame_id_ = LEFT_ELBOW_FRAME;
//  transform.stamp_ = ros::Time::now ();
//  tf_broadcaster->sendTransform (transform);

  btVector3 trans_left_shoulder = frame_transforms[LEFT_SHOULDER_FRAME].getOrigin ();
  std::cout <<"shoulder: "<< trans_left_shoulder.x () << " " << trans_left_shoulder.y () << " " << trans_left_shoulder.z ()
      << std::endl;
  btVector3 trans_left_elbow = frame_transforms[LEFT_ELBOW_FRAME].getOrigin ();
  std::cout <<"elbow   : "<< trans_left_elbow.x () << " " << trans_left_elbow.y () << " " << trans_left_elbow.z () << std::endl;
  joint_positions[TORSO_L_SHOULDER_X] = trans_left_shoulder.getX ();
  joint_positions[L_SHOULDER_X_L_SHOULDER_Y] = trans_left_shoulder.getY ();
  joint_positions[L_SHOULDER_Y_L_SHOULDER_Z] = trans_left_shoulder.getZ ();
  joint_positions[L_SHOULDER_L_ELBOW_X] = trans_left_elbow.getX ();
  joint_positions[L_ELBOW_X_L_ELBOW_Y] = trans_left_elbow.getY ();
  joint_positions[L_ELBOW_Y_L_ELBOW_Z] = trans_left_elbow.getZ ();

  pub_human_state->publishTransforms (joint_positions, ros::Time::now ());

  //  if (healthy_transformation[LEFT_SHOULDER_FRAME] && healthy_transformation[RIGHT_SHOULDER_FRAME])
  //  {
  //    btVector3 trans_left_shoulder = frame_transforms[LEFT_SHOULDER_FRAME].getOrigin ();
  //    btVector3 trans_right_shoulder = frame_transforms[RIGHT_SHOULDER_FRAME].getOrigin ();
  //    //    std::cout<<(float)trans_left_shoulder.distance(trans_right_shoulder)<<std::endl;
  //  }
  //
  //  if (healthy_transformation[LEFT_SHOULDER_FRAME] && healthy_transformation[TORSO_FRAME])
  //  {
  //    btVector3 trans_left_shoulder = frame_transforms[LEFT_SHOULDER_FRAME].getOrigin ();
  //    btVector3 trans_torso = frame_transforms[TORSO_FRAME].getOrigin ();
  //    std::cout << (float)trans_left_shoulder.distance (trans_torso) << std::endl;
  //  }

  //  if (transformation_healthy)
  //  {
  //    double roll, pitch, yaw;
  //    extractTorsoJointsPositions (frame_transforms, roll, pitch, yaw);
  //    pub_human_state->publishTransforms(joint_positions, ros::Time::now());
  //  }
}
