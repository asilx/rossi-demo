//ros includes
#include <ros/ros.h>

#include "aff_msgs/ExperimentState.h"
#include "aff_msgs/ModuleStates.h"
#include "aff_msgs/Speech.h"
#include "aff_msgs/Features.h"
#include "feature_manager/Perception.h"
#include "/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/humanoid_aff_learning/icub_aff_learning/behavior_manager/srv_gen/cpp/include/behavior_manager/Action.h"
#include "tabletop_2D_segmentation/Perception2D.h"

#include "sound_play/SoundRequest.h"

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

#include "iostream"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

const string port_in_ext_motion_det_name = "/ext_motion_detection:i";
const string port_out_ext_motion_det_name = "/ext_motion_detection:o";

const string port_in_int_motion_det_name = "/int_motion_detection:i";
const string port_out_int_motion_det_name = "/int_motion_detection:o";

const string ask_for_action = "Now what";
const string ask_for_effect = "What has just happened";
const string ask_for_action_persistent = "Please, tell me what to do";
const string ask_for_effect_persistent = "Please, tell me what happened";
const string intro_speech = "Hi, let's rock and roll !";

const bool USE_EXT_MOTION_DETECTION = false;
const bool USE_INT_MOTION_DETECTION = false;

const float TABLE_REGION_WIDTH = 0.33;
const float TABLE_REGION_HEIGHT = 0.23;
const float TABLE_X_SHIFT = -0.87;
const float TABLE_Y_SHIFT = -0.52;

bool workspace_detection_active = false;
uint exp_state_;
uint prev_exp_state_ = aff_msgs::ExperimentState::STANDBY;

ros::NodeHandle* n;
ros::Publisher pub_exp_state_;
ros::Subscriber sub_speech_cmd_;
ros::Publisher pub_say_;
ros::Publisher pub_labels_;
//BufferedPort<Bottle> port_out_say_;
//Port port_rpc_say_state_;

feature_manager::Perception srv_perception;
tabletop_2D_segmentation::Perception2D srv_perception_2D;
behavior_manager::Action srv_action;
sound_play::SoundRequest msg_speech_out;
aff_msgs::Speech msg_speech_in;

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

bool new_speech_cmd_rcvd_ = false;

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

  pub_labels_ = n->advertise<aff_msgs::Features> ("/experiment_labels", 2);

  exp_state_ = aff_msgs::ExperimentState::ASK_FOR_ACTION;
  //  exp_state_ = aff_msgs::ExperimentState::ACTION;

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

  say (intro_speech, 3);
}

//void
//say2 (const string& what_to_say)
//{
//  std::cout << "saying" << std::endl;
//
//  Bottle& b = port_out_say_.prepare ();
//  b.clear ();
//  b.addString (what_to_say.c_str ());
//  port_out_say_.write ();
//
//  Bottle cmd, reply;
//  cmd.addVocab (VOCAB4('s','t','a','t'));
//  //wait until speak is finished
//  bool go_on;
//  do
//  {
//    go_on = port_rpc_say_state_.write (cmd, reply);
//    if (!(go_on && (reply.get (0).asString () == "speaking")))
//      go_on = false;
//
//    std::cout << reply.get (0).asString () << std::endl;
//    ros::spinOnce ();
//  } while (!go_on && n->ok ());
//}

void getObjectRegionIndices(std::vector<float> object_center, uint& ooi_row, uint &ooi_col)
{
  float x = object_center[0];
  float y = object_center[1];

  x -= TABLE_X_SHIFT;
  y -= TABLE_Y_SHIFT;

  ooi_row  = floor(object_center[0]/TABLE_REGION_WIDTH);
  ooi_col = floor(object_center[1]/TABLE_REGION_HEIGHT);
}

void
run ()
{
  bool action_asked_for_first = true;
  bool effect_asked_for_first = true;
  bool action_asked_for = false;
  bool effect_asked_for = false;
  ros::Time t1;

  uint region_row=0;
  uint region_col=0;

  ROS_INFO("experiment is starting");
  
  std::cout << "First action is on the house." << std::endl;
  
  bool firstRun = true;
  bool objectPercepted = false;
  bool act = true;
  int in_x;
  
  double prev_y = 0;
  double total = 0;
  while (n->ok ())
  {
  
  	if(firstRun)
  	{
  		std::cout << "First run is about to commence. Press an integer key to continue" << std::endl;
  		std::cin >> in_x;		
  		exp_state_ = aff_msgs::ExperimentState::PERCEPTION;
		srv_action.request.task = behavior_manager::Action::Request::LOOK_AT_FACE;
		std::cout << "calling look at face action service" << std::endl;
		ros::service::call ("/action", srv_action.request, srv_action.response);

		ROS_INFO("calling -tuck_arms- action service2");
		srv_action.request.task = behavior_manager::Action::Request::TUCK_ARMS;
		ros::service::call ("/action", srv_action.request, srv_action.response);
		ROS_INFO("finished -tuck_arms- action2");
  		
  		msg_speech_in.speech_cmd = behavior_manager::Action::Request::PUSH_LEFT;
  		msg_speech_in.speech_arg = 0;
  		firstRun = false;
  		
  		
  		
  	}
  
  
    switch (exp_state_)
    {
      case aff_msgs::ExperimentState::ASK_FOR_ACTION:
        //        ROS_INFO ("*** ASK FOR ACTION ! ***");
        if (!action_asked_for)
        {
          t1 = ros::Time::now ();
          action_asked_for = true;
          if (action_asked_for_first)
            say (ask_for_action);
          else
            say (ask_for_action_persistent);
          ros::spinOnce ();
        }
        else
        {
          ros::Duration time_passed = ros::Time::now () - t1;
          if (time_passed.toSec () > 8.0)
          {
            action_asked_for = false;
            action_asked_for_first = false;
          }
        }

        if (new_speech_cmd_rcvd_)
        {
          new_speech_cmd_rcvd_ = false;
          if (msg_speech_in.speech_cmd < aff_msgs::Speech::MAX_ACTION_INDEX)
          {
            std::cout << "Action is no longer asked for" << std::endl;
            //action speech command is received, now percept the environment
            action_asked_for = false;//clear the flag

            exp_state_ = aff_msgs::ExperimentState::PERCEPTION;
          }
          else
          {
            ROS_WARN("speech command index out of action enumerations, discarding the message! ");
          }
        }
        break;
      case aff_msgs::ExperimentState::PERCEPTION:
      
        //        ROS_INFO ("*** PERCEPTION ! ***");
        //send behavior_manager a message so that robot goes for a configuration
        //which is required for the perception process (e.g. tuck_arms)
        ROS_INFO("calling -tuck_arms- action service3");
        //srv_action.request.task = behavior_manager::Action::Request::TUCK_ARMS;
        ros::service::call ("/action", srv_action.request, srv_action.response);
        ROS_INFO("finished -tuck_arms- action3");

        //Assuming that this action is done successfully, now call perception service
        //and ask for the center and dims of the requested object

        ros::Duration(5).sleep();

        ROS_INFO("calling perception 3D service");

        srv_perception.request.task = feature_manager::Perception::Request::DO_PERCEPT;
        srv_perception.request.arg = msg_speech_in.speech_arg;//label of the object of interest
        ros::service::call ("/perception", srv_perception.request, srv_perception.response);
        ROS_INFO ("finished perception 3D");
        
        ROS_INFO("calling perception 2D service");
        srv_perception_2D.request.task = tabletop_2D_segmentation::Perception2D::Request::DO_PERCEPT;
        //TODO: extract table region row and col values of ooi
        //then send them via service request
        // msg_speech_in.speech_arg;//label of the object of interest
        getObjectRegionIndices(srv_perception.response.pushable_object_center, region_row, region_col);
        std::cout<<region_row<<" "<<region_col<<std::endl;
        srv_perception_2D.request.arg = region_row;
        srv_perception_2D.request.arg2 = region_col;
        //first look at object
        srv_action.request.task = behavior_manager::Action::Request::LOOK_AT_REGION;
        srv_action.request.arg = region_row*3 + region_col;
        cout << "here" << endl;
        ros::service::call("/action", srv_action.request, srv_action.response);
	cout << "here percept 2d" << endl;
  	
        ros::service::call ("/perception2D", srv_perception_2D.request, srv_perception_2D.response);
        ROS_INFO ("finished perception 2D");
        
        
        if(!objectPercepted)
      	{
      		prev_y = srv_perception.response.pushable_object_center[1];
      		objectPercepted = true;	
      	}
        
        exp_state_ = aff_msgs::ExperimentState::ACTION;
        break;
      case aff_msgs::ExperimentState::ACTION:
        //        ROS_INFO("*** ACTION ! ***\n");

        srv_action.request.task = msg_speech_in.speech_cmd;
	msg_speech_in.speech_arg = 0;
        std::cout << "Sending action with task id: " << srv_action.request.task << std::endl;

//        srv_action.request.task = 1;
        srv_action.request.pushable_object_center = srv_perception.response.pushable_object_center;
        srv_action.request.pushable_object_size = srv_perception.response.pushable_object_size;

        if( srv_action.request.task == aff_msgs::Speech::PUSH_LEFT)
        {
        	firstRun = false;
        	exp_state_ = aff_msgs::ExperimentState::PERCEPTION;
        	//if(prev_y != 0) 
        	//{
        		total += abs(srv_perception.response.pushable_object_center[1] - prev_y); 
        		cout << "Total displacement recorded as: " << total << endl;
        	//}
        	prev_y = srv_perception.response.pushable_object_center[1];
        	if (total >= 0.08) 
        	{
        		exp_state_ = aff_msgs::ExperimentState::ASK_FOR_EFFECT;
        		cout << "Now it is time to ask for effect!" << endl;
        		act = false;
        		break;
        	}
        }
        else
        	exp_state_ = aff_msgs::ExperimentState::ASK_FOR_EFFECT;
        	
        	
        if (act)
        {
		ROS_INFO ( "calling action service");
		ros::service::call ("/action", srv_action.request, srv_action.response);
		ROS_INFO ("finished action");	
        }
        break;
      case aff_msgs::ExperimentState::ASK_FOR_EFFECT:
        //        ROS_INFO ("*** ASK FOR EFFECT ! ***");
        if (!effect_asked_for)
        {
          t1 = ros::Time::now ();
          effect_asked_for = true;
          if (effect_asked_for_first)
            say (ask_for_effect);
          else
            say (ask_for_effect_persistent);
          ros::spinOnce ();
        }
        else
        {
          ros::Duration time_passed = ros::Time::now () - t1;
          if (time_passed.toSec () > 8.0)
          {
            effect_asked_for = false;
            effect_asked_for_first = false;
          }
        }
        if (new_speech_cmd_rcvd_)
        {
          new_speech_cmd_rcvd_ = false;
          if (aff_msgs::Speech::MAX_ACTION_INDEX < msg_speech_in.speech_cmd && msg_speech_in.speech_cmd
              < aff_msgs::Speech::MAX_EFFECT_INDEX)
          {
            //effect speech command is received, now percept the environment
            std::cout << "Effect no longer asked!" << std::endl;
            effect_asked_for = false;//clear the flag
            exp_state_ = aff_msgs::ExperimentState::EFFECT;
          }
        }
        break;
      case aff_msgs::ExperimentState::EFFECT:
        //        ROS_INFO ("*** EFFECT ! ***\n");
        //send behavior_manager a message so that robot goes for a configuration
        //which is required for the perception process (e.g. tuck_arms)
        ROS_INFO("calling -tuck_arms- action service");
        srv_action.request.task = behavior_manager::Action::Request::TUCK_ARMS;
        ros::service::call ("/action", srv_action.request, srv_action.response);
        ROS_INFO("finished -tuck_arms- action");

        ros::Duration(5).sleep();
        //Assuming that this action is done successfully, now call perception service
        srv_perception.request.task = feature_manager::Perception::Request::EXTRACT_EFFECT;
        srv_perception.request.arg = msg_speech_in.speech_cmd;
        ROS_INFO ("calling perception service");
        ros::service::call ("/perception", srv_perception.request, srv_perception.response);
        ROS_INFO ("finished perception");

        action_asked_for_first = false;
        effect_asked_for_first = false;

        exp_state_ = aff_msgs::ExperimentState::ASK_FOR_ACTION;
        objectPercepted = false;
        act = true;

        break;
      default:
        break;
    }
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
