#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"

//1-listen tfs published by openni_tracker
//2-forward these tfs to the gazebo

const std::string BASE_FRAME = "base_footprint";
const std::string OPENNI_DEPTH_FRAME = "openni_depth_frame";

const std::string NECK_FRAME = "neck_1";
const std::string TORSO_FRAME = "torso_1";
const std::string HEAD_FRAME = "head_1";

const std::string LEFT_SHOULDER_FRAME = "left_shoulder_1";
const std::string RIGHT_SHOULDER_FRAME = "right_shoulder_1";
const std::string LEFT_ELBOW_FRAME = "left_elbow_1";
const std::string RIGHT_ELBOW_FRAME = "right_elbow_1";
const std::string LEFT_HAND_FRAME = "left_hand_1";
const std::string RIGHT_HAND_FRAME = "right_hand_1";

const std::string LEFT_LOWER_ARM_FRAME = "left_lower_arm";
const std::string RIGHT_LOWER_ARM_FRAME = "right_lower_arm";
const std::string LEFT_UPPER_ARM_FRAME = "left_upper_arm";
const std::string RIGHT_UPPER_ARM_FRAME = "right_upper_arm";

const std::string LEFT_HIP_FRAME = "left_hip_1";
const std::string RIGHT_HIP_FRAME = "right_hip_1";
const std::string LEFT_KNEE_FRAME = "left_knee_1";
const std::string RIGHT_KNEE_FRAME = "right_knee_1";
const std::string LEFT_FOOT_FRAME = "left_foot_1";
const std::string RIGHT_FOOT_FRAME = "right_foot_1";

const std::string HUMAN_NAMESPACE = "human::";

tf::TransformListener* listener;
ros::Publisher pub_link_st;
ros::Publisher pub_model_st;
ros::Subscriber sub_link_st;
ros::Subscriber sub_model_st;
std::vector<std::string> frame_names;

void
init (std::map<std::string, tf::StampedTransform>& frame_transforms);

void
update (std::map<std::string, tf::StampedTransform>& frame_transforms);

void
linkStateCallback (gazebo_msgs::LinkStates::ConstPtr msg_link_st)
{
  //  for (uint i = 0; i < msg_link_st->name.size (); i++)
  //    std::cout << msg_link_st->name[i] << std::endl;
}

void
modelStateCallback (gazebo_msgs::ModelStates::ConstPtr msg_model_st)
{
  //    std::cout << "---------------" << std::endl;
  //    for (uint i = 0; i < msg_model_st->name.size (); i++)
  //    {
  //      if (!msg_model_st->name[i].compare ("human"))
  //      {
  //        std::cout << msg_model_st->name[i] << std::endl;
  //        std::cout << msg_model_st->pose[i] << std::endl;
  //        //            std::cout << msg_model_st->twist[i] << std::endl;
  //      }
  //    }
  //    std::cout << "---------------" << std::endl;
}

int
main (int argc, char* argv[])
{
  ros::init (argc, argv, "gazebo_st_publisher");
  ros::NodeHandle n;

  listener = new tf::TransformListener ();
  pub_link_st = n.advertise<gazebo_msgs::LinkState> ("/gazebo/set_link_state", 50);
  pub_model_st = n.advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 50);
  //  sub_link_st = n.subscribe<gazebo_msgs::LinkStates> ("/gazebo/link_states", 100, &linkStateCallback);
  //  sub_model_st = n.subscribe<gazebo_msgs::ModelStates> ("/gazebo/model_states", 100, &modelStateCallback);

  std::map<std::string, tf::StampedTransform> frame_transforms;
  init (frame_transforms);

  ros::Rate r (50);
  while (n.ok ())
  {
    update (frame_transforms);
    ros::spinOnce ();
    r.sleep ();
  }

  return 1;
}

void
init (std::map<std::string, tf::StampedTransform>& frame_transforms)
{
  frame_names.push_back (TORSO_FRAME);
  frame_names.push_back (HEAD_FRAME);
  frame_names.push_back (NECK_FRAME);
  frame_names.push_back (LEFT_SHOULDER_FRAME);
  frame_names.push_back (RIGHT_SHOULDER_FRAME);
  frame_names.push_back (LEFT_ELBOW_FRAME);
  frame_names.push_back (RIGHT_ELBOW_FRAME);
  frame_names.push_back (LEFT_HAND_FRAME);
  frame_names.push_back (RIGHT_HAND_FRAME);
  frame_names.push_back (LEFT_UPPER_ARM_FRAME);
  frame_names.push_back (RIGHT_UPPER_ARM_FRAME);
  frame_names.push_back (LEFT_LOWER_ARM_FRAME);
  frame_names.push_back (RIGHT_LOWER_ARM_FRAME);

  //  frame_names[6] = LEFT_HIP_FRAME;
  //  frame_names[7] = RIGHT_HIP_FRAME;
  //  frame_names[8] = LEFT_KNEE_FRAME;
  //  frame_names[9] = RIGHT_KNEE_FRAME;
  //  frame_names[10] = LEFT_FOOT_FRAME;
  //  frame_names[11] = RIGHT_FOOT_FRAME;
}

void
update (std::map<std::string, tf::StampedTransform>& frame_transforms)
{
  bool transformation_healthy = true;
  tf::StampedTransform transform;
  try
  {
    listener->lookupTransform (BASE_FRAME.c_str (), TORSO_FRAME.c_str (), ros::Time (0), transform);
    frame_transforms[TORSO_FRAME] = transform;
//    std::cout << transform.getOrigin ().x () << " " << transform.getOrigin ().y () << " "
//        << transform.getOrigin ().z () << std::endl;
    //      healthy_transformation[frame_names[i]] = true;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    //      healthy_transformation[frame_names[i]] = false;
    transformation_healthy = false;
  }

  for (uint i = 1; i < frame_names.size (); i++)
  {
    try
    {
      listener->lookupTransform (TORSO_FRAME.c_str (), frame_names[i].c_str (), ros::Time (0), transform);
      frame_transforms[frame_names[i]] = transform;
//      std::cout << transform.getOrigin ().x () << " " << transform.getOrigin ().y () << " "
//          << transform.getOrigin ().z () << std::endl;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      transformation_healthy = false;
    }
  }

  if (transformation_healthy)
  {
    //    std::cout<<"****************"<<std::endl;
    for (uint i = 0; i < frame_names.size (); i++)
    {
      if (!frame_names[i].compare (TORSO_FRAME))
      {
        gazebo_msgs::ModelState msg_model_st;
        msg_model_st.model_name = "human";
        msg_model_st.pose.position.x = frame_transforms[TORSO_FRAME].getOrigin ().x ();
        msg_model_st.pose.position.y = frame_transforms[TORSO_FRAME].getOrigin ().y ();
        msg_model_st.pose.position.z = frame_transforms[TORSO_FRAME].getOrigin ().z ();

        msg_model_st.pose.orientation.x = frame_transforms[TORSO_FRAME].getRotation ().x ();
        msg_model_st.pose.orientation.y = frame_transforms[TORSO_FRAME].getRotation ().y ();
        msg_model_st.pose.orientation.z = frame_transforms[TORSO_FRAME].getRotation ().z ();
        msg_model_st.pose.orientation.w = frame_transforms[TORSO_FRAME].getRotation ().w ();

        msg_model_st.twist.angular.x = 0;
        msg_model_st.twist.angular.y = 0;
        msg_model_st.twist.angular.z = 0;
        msg_model_st.twist.linear.x = 0;
        msg_model_st.twist.linear.y = 0;
        msg_model_st.twist.linear.z = 0;

        msg_model_st.reference_frame = "world";
        if (pub_model_st.getNumSubscribers ())
          pub_model_st.publish (msg_model_st);
        ros::spinOnce ();

        gazebo_msgs::LinkState msg_link_st;
        msg_link_st.link_name = HUMAN_NAMESPACE + TORSO_FRAME;
        msg_link_st.pose = msg_model_st.pose;
        msg_link_st.twist = msg_model_st.twist;
        msg_link_st.reference_frame = "world";

        if (pub_link_st.getNumSubscribers ())
          pub_link_st.publish (msg_link_st);
      }

      for (uint i = 1; i < frame_names.size (); i++)
      {
        gazebo_msgs::LinkState msg_link_st;
        msg_link_st.link_name = HUMAN_NAMESPACE + frame_names[i];
//        std::cout << msg_link_st.link_name << std::endl;
        msg_link_st.pose.position.x = frame_transforms[frame_names[i]].getOrigin ().x ();
        msg_link_st.pose.position.y = frame_transforms[frame_names[i]].getOrigin ().y ();
        msg_link_st.pose.position.z = frame_transforms[frame_names[i]].getOrigin ().z ();

        msg_link_st.pose.orientation.x = frame_transforms[frame_names[i]].getRotation ().x ();
        msg_link_st.pose.orientation.y = frame_transforms[frame_names[i]].getRotation ().y ();
        msg_link_st.pose.orientation.z = frame_transforms[frame_names[i]].getRotation ().z ();
        msg_link_st.pose.orientation.w = frame_transforms[frame_names[i]].getRotation ().w ();

        msg_link_st.twist.angular.x = 0;
        msg_link_st.twist.angular.y = 0;
        msg_link_st.twist.angular.z = 0;
        msg_link_st.twist.linear.x = 0;
        msg_link_st.twist.linear.y = 0;
        msg_link_st.twist.linear.z = 0;

        msg_link_st.reference_frame = "human::torso_1";
        //        std::cout << msg_link_st.reference_frame << std::endl;
        if (pub_link_st.getNumSubscribers ())
        {
          pub_link_st.publish (msg_link_st);
          ros::spinOnce ();
        }
      }
    }
    //    std::cout<<"****************"<<std::endl;
  }
}
