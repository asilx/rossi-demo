/*
 * test_perceptors.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: kadir
 */

//ros includes
#include <ros/ros.h>

#include "aff_msgs/Speech.h"
#include "aff_msgs/ExperimentState.h"
#include "feature_manager/Perception.h"
#include "tabletop_2D_segmentation/Perception2D.h"
#include "/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/humanoid_aff_learning/icub_aff_learning/behavior_manager/srv_gen/cpp/include/behavior_manager/Action.h"

// yarp includes
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <pthread.h>

// standard includes
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "cv_bridge/cv_bridge.h"
#include "cv_bridge/CvBridge.h"
#include "sensor_msgs/Image.h"

#include "sound_play/SoundRequest.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

const string port_in_ext_motion_det_name = "/ext_motion_detection:i";
const string port_out_ext_motion_det_name = "/ext_motion_detection:o";

const string port_in_int_motion_det_name = "/int_motion_detection:i";
const string port_out_int_motion_det_name = "/int_motion_detection:o";

const bool USE_EXT_MOTION_DETECTION = false;
const bool USE_INT_MOTION_DETECTION = false;

const float TABLE_REGION_WIDTH = 0.33;
const float TABLE_REGION_HEIGHT = 0.23;
const float TABLE_X_SHIFT = -0.87;
const float TABLE_Y_SHIFT = -0.52;

const std::string WORKSPACE_2D_SEGMENTATION_SRV_NAME = "/tabletop_2D_segmentation";

const string ask_for_action = "Now what";
const string ask_for_effect = "What has just happened";
const string ask_for_action_persistent = "Please, tell me what to do";
const string ask_for_effect_persistent = "Please, tell me what happened";
const string intro_speech = "Hi, let's rock and roll !";
const string info_percepting = "I'm percepting";
const string info_acting = "I'm acting";
const string info_extracting = "I'm extracting effect";
const string info_human = "Now, you can change the table";

bool workspace_detection_active = false;
uint8_t exp_state_;

ros::NodeHandle* n;
ros::Publisher pub_exp_state_;
ros::Subscriber sub_speech_cmd_;
ros::Publisher pub_say_;

feature_manager::Perception srv_perception;
tabletop_2D_segmentation::Perception2D srv_perception_2D;
behavior_manager::Action srv_action;
sound_play::SoundRequest msg_speech_out;
aff_msgs::Speech msg_speech_in;
int object_id_ = 0;

sensor_msgs::CvBridge* bridge_;

bool new_speech_cmd_rcvd_ = false;

class DataPort : public BufferedPort<Bottle>
{
  virtual void
  onRead (Bottle& b)
  {
    //external motion detection port received data
    if (this->getName () == port_in_ext_motion_det_name.c_str ())
    {
      int data = b.get (0).asInt ();
      if (data == 0)//no external motion, workspace detection is active
      {
        ROS_INFO("External motion vanished ! Data acquisition is resuming");
        workspace_detection_active = true;
      }
      else
      {
        ROS_INFO("External motion detected ! Data acquisition is suspended");
        workspace_detection_active = false;
      }
    }
    else if (this->getName () == port_in_int_motion_det_name.c_str ())
    {
      int data = b.get (0).asInt ();
      if (data == 0)//no internal motion, workspace detection is active
      {
        ROS_INFO("Internal motion vanished ! Data acquisition is resuming");
        workspace_detection_active = true;
      }
      else
      {
        ROS_INFO("Internal motion detected ! Data acquisition is suspended");
        workspace_detection_active = false;
      }
    }
  }
};

DataPort port_in_ext_motion_det;
DataPort port_in_int_motion_det;

void
nullDeleter (void*)
{
}

void
publishExpState ()
{
  aff_msgs::ExperimentState s;
  s.experiment_state = exp_state_;
  pub_exp_state_.publish (s);
}

void
speechCmdCallback (aff_msgs::SpeechConstPtr speech_cmd)
{
  if (speech_cmd->speech_cmd >= 0)
  {
    new_speech_cmd_rcvd_ = true;
    msg_speech_in = *speech_cmd;
    std::cout << "I have received a message..." << std::endl;
    std::cout << (int)msg_speech_in.speech_cmd << " " << (int)msg_speech_in.speech_arg << std::endl;
  }
  else
  {
    ROS_DEBUG("speech cmd -1 is received, discarding message!");
  }

}

void
say (const string& what_to_say, double speech_t_length = 2)
{
  std::cout << "saying " << what_to_say << std::endl;
  msg_speech_out.sound = sound_play::SoundRequest::SAY;
  msg_speech_out.command = sound_play::SoundRequest::PLAY_ONCE;
  msg_speech_out.arg = what_to_say.c_str ();
  pub_say_.publish (msg_speech_out);
  ros::Duration (speech_t_length).sleep ();
  ros::spinOnce ();
}

void
init ()
{
  //ros initializations
  n = new ros::NodeHandle ();
  pub_exp_state_ = n->advertise<aff_msgs::ExperimentState> ("/experiment_state", 10);
  pub_say_ = n->advertise<sound_play::SoundRequest> ("/robotsound", 2);
  sub_speech_cmd_ = n->subscribe<aff_msgs::Speech> ("/speech_command", 2, &speechCmdCallback);

  exp_state_ = aff_msgs::ExperimentState::ASK_FOR_ACTION;
  //  exp_state_ = aff_msgs::ExperimentState::ACTION;

  bridge_ = new sensor_msgs::CvBridge ();

  //yarp initializations
  Network::init ();

  if (USE_EXT_MOTION_DETECTION)
  {
    port_in_ext_motion_det.useCallback ();
    port_in_ext_motion_det.open (port_in_ext_motion_det_name.c_str ());
    Network::connect (port_out_ext_motion_det_name.c_str (), port_in_ext_motion_det_name.c_str ());
    std::cout << "waiting for an -external motion detection- port to be connected" << std::endl;
    while (!Network::isConnected (port_out_ext_motion_det_name.c_str (), port_in_ext_motion_det_name.c_str ())
        && n->ok ())
    {
    }
    std::cout << "An -external motion detection- port is connected, ready for activation data..." << std::endl;
  }

  if (USE_INT_MOTION_DETECTION)
  {
    port_in_int_motion_det.useCallback ();
    port_in_int_motion_det.open (port_in_int_motion_det_name.c_str ());
    Network::connect (port_out_int_motion_det_name.c_str (), port_in_int_motion_det_name.c_str ());
    std::cout << "waiting for an -internal motion detection- port  to be connected" << std::endl;
    while (!Network::isConnected (port_out_int_motion_det_name.c_str (), port_in_int_motion_det_name.c_str ())
        && n->ok ())
    {
    }
    std::cout << "An -internal motion detection- port is connected, ready for activation data..." << std::endl;
  }
}

void
getObjectRegionIndices (const std::vector<float>& object_center, int& ooi_row, int &ooi_col)
{
  float x = object_center[0];
  float y = object_center[1];

  x -= TABLE_X_SHIFT;
  y -= TABLE_Y_SHIFT;

  std::cout << x << " " << y << std::endl;

  ooi_row = floor (x / TABLE_REGION_HEIGHT);
  ooi_col = floor (y / TABLE_REGION_WIDTH);

  std::cout << ooi_row << " " << ooi_col << std::endl;
}

void
run ()
{
  bool action_asked_for_first = true;
  bool effect_asked_for_first = true;
  bool action_asked_for = false;
  bool effect_asked_for = false;
  ros::Time t1;

  int region_row = 0;
  int region_col = 0;

  //  cvNamedWindow ("asd");

  ROS_INFO("experiment is starting");
  say (intro_speech, 3);
  ros::spinOnce ();

//  ROS_INFO("calling -tuck_arms- action service");
//  srv_action.request.task = behavior_manager::Action::Request::TUCK_ARMS;
//  ros::service::call ("/action", srv_action.request, srv_action.response);
//  ROS_INFO("finished -tuck_arms- action");

  say (intro_speech, 3);
  publishExpState ();
  ros::spinOnce ();

  while (n->ok ())
  {

    //call 3D service call

    say (info_percepting);
    ROS_INFO("calling perception 3D service");
    srv_perception.request.task = feature_manager::Perception::Request::DO_PERCEPT;
    srv_perception.request.arg = 0;
    object_id_ = srv_perception.request.arg;
    ros::service::call ("/perception", srv_perception.request, srv_perception.response);
    ROS_INFO ("finished perception 3D");

    //look at the object
    srv_action.request.task = behavior_manager::Action::Request::LOOK_AT_POINT;
    std::cout << srv_perception.response.pushable_object_center[0] << " "
        << srv_perception.response.pushable_object_center[1] << " "
        << srv_perception.response.pushable_object_center[2] << std::endl;
//    int i;
//    cin >>i;
    srv_action.request.pushable_object_center = srv_perception.response.pushable_object_center;
    ros::service::call ("/action", srv_action.request, srv_action.response);

    ros::spinOnce ();
  }
}

void
fini ()
{

  //  port_rpc_say_state_.close ();
  //  port_out_say_.close ();
  port_in_ext_motion_det.close ();
  port_in_int_motion_det.close ();

  Network::fini ();
}

int
main (int argc, char** argv)
{
  //ros initializations
  ros::init (argc, argv, "experiment_manager");
  init ();
  run ();
  fini ();

  return 0;
}
