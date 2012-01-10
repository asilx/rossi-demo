#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>

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

const std::string TORSO_ROLL_JOINT = "torso_roll_joint";
const std::string TORSO_PITCH_JOINT = "torso_roll_joint";
const std::string TORSO_YAW_JOINT = "torso_roll_joint";
const std::string L_SHOULDER_JOINT = "l_shoulder_joint";
const std::string R_SHOULDER_JOINT = "r_shoulder_joint";
const std::string L_ELBOW_JOINT = "l_elbow_joint";
const std::string R_ELBOW_JOINT = "r_elbow_joint";
const std::string L_HAND_JOINT = "l_hand_joint";
const std::string R_HAND_JOINT = "r_hand_joint";
const std::string NECK_JOINT = "neck_joint";

//std::map<std::string, bool> healthy_transformation;
std::vector<std::string> frame_names (15);
std::vector<std::string> joint_names (10);
tf::TransformListener* listener;
std::map<std::string, double> joint_positions;
robot_state_publisher::RobotStatePublisher* pub_human_state;
KDL::Tree* tree_human;

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

  std::string robot_desc_string;
  node.param ("robot_description", robot_desc_string, std::string ());
  if (!kdl_parser::treeFromString (robot_desc_string, *tree_human))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  pub_human_state = new robot_state_publisher::RobotStatePublisher (*tree_human);
  KDL::SegmentMap seg_map = tree_human->getSegments ();
  std::map<std::string, KDL::TreeElement>::iterator it;
  it = seg_map.begin();


  for(uint i=0;i<seg_map.size();i++)
    std::cout<<it->first<<std::endl;

//  KDL::TreeElement elem = seg_map[TORSO_ROLL_JOINT];
  //  elem.segment.
  //  seg_map.
  //  pub_human_state->publishTransforms()
  //  listener.waitForTransform()

  ros::Rate rate (10.0);
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

  joint_names[0] = TORSO_ROLL_JOINT;
  joint_names[1] = TORSO_PITCH_JOINT;
  joint_names[2] = TORSO_YAW_JOINT;
  joint_names[3] = L_SHOULDER_JOINT;
  joint_names[4] = R_SHOULDER_JOINT;
  joint_names[5] = L_ELBOW_JOINT;
  joint_names[6] = R_ELBOW_JOINT;
  joint_names[7] = L_HAND_JOINT;
  joint_names[8] = R_HAND_JOINT;
  joint_names[9] = NECK_JOINT;

  //  for (uint i = 0; i < frame_names.size (); i++)
  //    healthy_transformation[frame_names[i]] = false;

  //  listener.getFrameStrings (frame_names);
  //  std::cout << "***" << std::endl;
  //  for (uint i = 0; i < frame_names.size (); i++)
  //    std::cout << frame_names[i] << std::endl;
  //  std::cout << "---" << std::endl;

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
      listener->lookupTransform ("/openni_depth_frame", frame_names[i].c_str (), ros::Time (0), transform);
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

  if (transformation_healthy)
  {
    double roll, pitch, yaw;
    extractTorsoJointsPositions (frame_transforms, roll, pitch, yaw);
    pub_human_state->publishTransforms(joint_positions, ros::Time::now());
  }
}

void
extractTorsoJointsPositions (std::map<std::string, tf::StampedTransform>& frame_transforms, double& roll,
                             double& pitch, double& yaw)
{
  tf::Transform A;
  A.setOrigin (btVector3 (0, 0, 0));
  A.setRotation (btQuaternion (btVector3 (0, 0, 1), 0));

  btVector3 mid_shoulder = (frame_transforms[LEFT_SHOULDER_FRAME].getOrigin ()
      + frame_transforms[RIGHT_SHOULDER_FRAME].getOrigin ()) / 2;

  btVector3 A_prime_y = frame_transforms[LEFT_SHOULDER_FRAME].getOrigin () - mid_shoulder;
  A_prime_y = A_prime_y.normalize ();

  btVector3 A_prime_z = mid_shoulder - frame_transforms[TORSO_FRAME].getOrigin ();
  A_prime_z = A_prime_z.normalize ();

  btVector3 A_prime_x = A_prime_y.cross (A_prime_z);
  A_prime_x = A_prime_x.normalize ();//just in case

  btMatrix3x3 A_prime_rot;
  A_prime_rot.setValue (A_prime_x.getX (), A_prime_y.getX (), A_prime_z.getX (), A_prime_x.getY (), A_prime_y.getY (),
                        A_prime_z.getY (), A_prime_x.getZ (), A_prime_y.getZ (), A_prime_z.getZ ());

  tf::Transform A_prime;
  A_prime.setBasis (A_prime_rot);
  A_prime.setOrigin (btVector3 (0, 0, 0));

  tf::Transform coord_transform;
  coord_transform.mult (A_prime, A.inverse ());

  //  double roll, pitch, yaw;
  coord_transform.getBasis ().getRPY (roll, pitch, yaw);

  ROS_DEBUG("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
  joint_positions[TORSO_ROLL_JOINT] = roll;
  joint_positions[TORSO_PITCH_JOINT] = pitch;
  joint_positions[TORSO_YAW_JOINT] = yaw;
}
