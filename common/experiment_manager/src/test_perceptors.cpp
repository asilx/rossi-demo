/*
 * test_perceptors.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: kadir
 */

//ros includes
#include <ros/ros.h>

#include "/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/common/aff_msgs/msg_gen/cpp/include/aff_msgs/ObjectOfInterest.h"
#include "/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/common/aff_msgs/msg_gen/cpp/include/aff_msgs/Speech.h"
#include "/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/common/aff_msgs/msg_gen/cpp/include/aff_msgs/ExperimentState.h"
//#include "feature_manager/Perception.h"
#include "al_srvs/Perception.h"
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

// old big ugly table
//const float TABLE_REGION_WIDTH = 0.33;
//const float TABLE_REGION_HEIGHT = 0.23;
//const float TABLE_X_SHIFT = -0.87;
//const float TABLE_Y_SHIFT = -0.52;

const float TABLE_REGION_WIDTH = 0.24;
const float TABLE_REGION_HEIGHT = 0.14;
const float TABLE_X_SHIFT = -0.58;
const float TABLE_Y_SHIFT = -0.35;

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
const string info_everything_ok = "Is everything okey ?";

std::vector<std::string> affordance_labels;
aff_msgs::ObjectOfInterest ooi_affordances;
ros::Publisher pub_aff_labels;

enum Affordances
{
  REACHABLE, TAKEABLE, GIVEABLE, GRASPABLE, VANISHABLE
};

bool workspace_detection_active = false;
uint8_t exp_state_;

ros::NodeHandle* n;
ros::Publisher pub_exp_state_;
ros::Subscriber sub_speech_cmd_;
ros::Publisher pub_say_;

al_srvs::Perception srv_perception;
tabletop_2D_segmentation::Perception2D srv_perception_2D;
behavior_manager::Action srv_action;
sound_play::SoundRequest msg_speech_out;
aff_msgs::Speech msg_speech_in;
int object_id_ = 0;
std::vector<float> pushable_object_center_ (3);

sensor_msgs::CvBridge* bridge_;

bool new_speech_cmd_rcvd_ = false;


int experimentEpoch = 0; // is updated at every step.

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


void incrementExperimentEpoch()
{
	experimentEpoch++;
	ROS_INFO("Experiment epoch raised");
	std::cout << "New epoch id: " << experimentEpoch << std::endl;
}

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
  pub_aff_labels = n->advertise<aff_msgs::ObjectOfInterest> ("/aff_labels", 10);

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

  // ++Onur
  bool callVisualPerceptors = true;
  // --Onur

  ros::Time t1;

  int region_row = 0;
  int region_col = 0;

  //  cvNamedWindow ("asd");


  std::cout << "First action is on the house." << std::endl;
  
  bool firstRun = true;
  int in_x;
  
  
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
  
  ROS_INFO("calling -tuck_arms- action service");
  srv_action.request.task = behavior_manager::Action::Request::TUCK_ARMS;
  ros::service::call ("/action", srv_action.request, srv_action.response);
  ROS_INFO("finished -tuck_arms- action");

  while (n->ok ())
  {
  	if(firstRun)
  	{
  		std::cout << "First run is about to commence. Press an integer key to continue (1: Reach, 2:PULL, 3:Push Left, 4:Push Right" << std::endl;
  		std::cout << "5:Push L Upper 6: Push R Upper 7: Grasp 8: Grasp Upper 9: Cover 10: Drop 11: Take 12:Give" << std::endl;
  		std::cin >> in_x;		
  		
		
		if (in_x != 10 && in_x != 11 && in_x != 12 && in_x != 13)
		{
			exp_state_ = aff_msgs::ExperimentState::PERCEPTION;
			srv_action.request.task = behavior_manager::Action::Request::LOOK_AT_FACE;
			std::cout << "calling look at face action service" << std::endl;
			ros::service::call ("/action", srv_action.request, srv_action.response);
		
			
		}

  		
  		if(in_x == 1)
  			msg_speech_in.speech_cmd = behavior_manager::Action::Request::REACH;
  		else if (in_x == 2)
  			msg_speech_in.speech_cmd = behavior_manager::Action::Request::PUSH_FORWARD;
  		else if (in_x == 3)
  			msg_speech_in.speech_cmd = behavior_manager::Action::Request::PUSH_LEFT;
  		else if (in_x == 4) msg_speech_in.speech_cmd = behavior_manager::Action::Request::PUSH_RIGHT;
  		else if (in_x == 5)
  			msg_speech_in.speech_cmd = behavior_manager::Action::Request::PUSH_LEFT_UPPER;
  		else if (in_x == 6) msg_speech_in.speech_cmd = behavior_manager::Action::Request::PUSH_RIGHT_UPPER;
  		else if (in_x == 7) msg_speech_in.speech_cmd = behavior_manager::Action::Request::GRASP;
  		else if (in_x == 8)
  			msg_speech_in.speech_cmd = behavior_manager::Action::Request::GRASP_UPPER;
  		else if (in_x == 9) msg_speech_in.speech_cmd = behavior_manager::Action::Request::COVER;
  		else if (in_x == 10)
  		{
  			exp_state_ = aff_msgs::ExperimentState::ACTION;
  			msg_speech_in.speech_cmd = behavior_manager::Action::Request::DROP;
  		}
  		else if (in_x == 11)
  		{
  			exp_state_ = aff_msgs::ExperimentState::ACTION;
  			msg_speech_in.speech_cmd = behavior_manager::Action::Request::TAKE;
  		}
  		else if (in_x == 12)
  		{
  			exp_state_ = aff_msgs::ExperimentState::ACTION;
  			msg_speech_in.speech_cmd = behavior_manager::Action::Request::GIVE;
  		}
  		else if (in_x == 13)
  		{
  			exp_state_ = aff_msgs::ExperimentState::ACTION;
  			msg_speech_in.speech_cmd = behavior_manager::Action::Request::DETECT_TOUCH;
  		}
  		cout << "object id?" << endl;
  		int id;
  		cin >> id;
  		msg_speech_in.speech_arg = id;
  		firstRun = false;
  		
  		publishExpState ();
  		
  		
  	}
    if (exp_state_ == aff_msgs::ExperimentState::LET_HUMAN_ACT)
    {
      //cvWaitKey(0);//wait for keyboard event
      int i;
      cin >> i;
      exp_state_ = aff_msgs::ExperimentState::ASK_FOR_ACTION;
      publishExpState ();
    }
    else if (exp_state_ == aff_msgs::ExperimentState::ASK_FOR_ACTION)
    {
      //      ROS_INFO ("*** ASK FOR ACTION ! ***");
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

          //if the action isn't related to any object in the world, then ask for a new action
          //by skipping anything perception related
          if (msg_speech_in.speech_cmd != aff_msgs::Speech::HOME && msg_speech_in.speech_cmd
              != aff_msgs::Speech::CANCEL)
          {
            //now call tuck arms and look at face to register face if any exists, and clear limbs from the table
            //TODO:
           // srv_action.request.experimentEpoch = experimentEpoch;
            srv_action.request.task = behavior_manager::Action::Request::LOOK_AT_FACE;
            std::cout << "calling look at face action service" << std::endl;
            ros::service::call ("/action", srv_action.request, srv_action.response);

            ROS_INFO("calling -tuck_arms- action service");
            srv_action.request.task = behavior_manager::Action::Request::TUCK_ARMS;
            ros::service::call ("/action", srv_action.request, srv_action.response);
            ROS_INFO("finished -tuck_arms- action");

            exp_state_ = aff_msgs::ExperimentState::PERCEPTION;
          }
          else
          {
            action_asked_for_first = false;
            effect_asked_for_first = false;

            exp_state_ = aff_msgs::ExperimentState::ASK_FOR_ACTION;
          }
          publishExpState ();
        }
        else
        {
          ROS_WARN("speech command index out of action enumerations, discarding the message! ");
        }
      }
    }
    else if (exp_state_ == aff_msgs::ExperimentState::PERCEPTION)
    {
      //send behavior_manager a message so that robot goes for a configuration
      //which is required for the perception process (e.g. tuck_arms)

      // ++Onur
      if (msg_speech_in.speech_cmd == behavior_manager::Action::Request::GIVE)
      {
        callVisualPerceptors = false;
        ROS_INFO("Visual perception callbacks are disabled");
      }
      // --Onur

      // ++Onur
      if (callVisualPerceptors)
      {
        say (info_percepting);
        ROS_INFO("calling perception 3D service");
        srv_perception.request.task = al_srvs::Perception::Request::DO_PERCEPT;
        srv_perception.request.arg = msg_speech_in.speech_arg;//label of the object of interest
        //srv_perception.request.experimentEpoch = 0;
        object_id_ = srv_perception.request.arg;
        if( ros::service::call ("/perception", srv_perception.request, srv_perception.response))
        	ROS_INFO ("finished perception 3D");
        else
        	ROS_WARN ("cukko");
        pushable_object_center_ = srv_perception.response.pushable_object_center;
        
        //now fixate to the object if any object interaction is commanded
        //an object interaction behavior is requested, hence 3D perception is forced to return an object-of-interest
        srv_action.request.task = behavior_manager::Action::Request::LOOK_AT_POINT;
    
        //srv_action.request.experimentEpoch = experimentEpoch;
        srv_action.request.pushable_object_center = srv_perception.response.pushable_object_center;
        std::cout << "calling look at point with" << srv_action.request.pushable_object_center[0] << " "
            << srv_action.request.pushable_object_center[1] << " " << srv_action.request.pushable_object_center[2]
            << std::endl;
        ros::service::call ("/action", srv_action.request, srv_action.response);

        /*ROS_INFO("calling perception 2D service");
        //srv_perception_2D.request.experimentEpoch = experimentEpoch;
        srv_perception_2D.request.task = tabletop_2D_segmentation::Perception2D::Request::DO_PERCEPT;
        getObjectRegionIndices (srv_perception.response.pushable_object_center, region_row, region_col);
        srv_perception_2D.request.arg = (int8_t)region_row;
        srv_perception_2D.request.arg2 = (int8_t)region_col;
        std::cout << (int)srv_perception_2D.request.arg << " " << (int)srv_perception_2D.request.arg2 << std::endl;
        ros::service::call (WORKSPACE_2D_SEGMENTATION_SRV_NAME, srv_perception_2D.request, srv_perception_2D.response);
        //            boost::shared_ptr<sensor_msgs::Image> img_ptr (&srv_perception_2D.response.raw_image, nullDeleter);
        //            cvShowImage ("asd", bridge_->imgMsgToCv (img_ptr, "bgr8"));
        //            cvWaitKey (0);
        ROS_INFO ("finished perception 2D");*/

        std::cout << "REACHABLE = 0" << std::endl;
        std::cout << "TAKEABLE  = 1" << std::endl;
        std::cout << "GIVEABLE  = 2" << std::endl;
        std::cout << "GRASPABLE = 3" << std::endl;
        std::cout << "VANISHABLE= 4" << std::endl;
        std::cout
            << "enter the affordance labels one by one: "<<std::endl;
        std::cout<<"e.g.<0 1 4 -1> for <reachable takeable vanishable> object, -1 is for quitting."
            << std::endl;

        int i = -1;
        std::vector<std::string> aff_str_labels;
        do
        {
          if (i < 5 && i >= 0)
          {
            aff_str_labels.push_back (affordance_labels[i]);
          }
        } while (n->ok () && i != -1);
        ooi_affordances.affordances = aff_str_labels;
        ooi_affordances.object_center = srv_perception.response.pushable_object_center;
        ooi_affordances.object_size = srv_perception.response.pushable_object_size;

        if (pub_aff_labels.getNumSubscribers () > 0)
          pub_aff_labels.publish (ooi_affordances);
      }

      // --Onur
      exp_state_ = aff_msgs::ExperimentState::ACTION;
      publishExpState ();
    }
    else if (exp_state_ == aff_msgs::ExperimentState::ACTION)
    {
      yarp::os::Network::disconnect("/o:ReflexCommand", "/tactGraspRight/rpc:i");
      yarp::os::Network::disconnect("/icub/skin/righthandcomp", "/i:ReflexTactile");
      yarp::os::Network::disconnect("/o:ReflexCommand", "/tactGraspLeft/rpc:i");
      yarp::os::Network::disconnect("/icub/skin/lefthandcomp", "/i:ReflexTactile");
    
      say (info_acting);
      ROS_INFO("ACTION!\n");

      //****************************************************

      //Debug mode: action will not be applied if the below is used.
      //ros::Duration (1).sleep ();

      //Experiment mode: action will be applied if the below is used
      srv_action.request.task = msg_speech_in.speech_cmd;

      std::cout << "Sending action with task id: " << srv_action.request.task << std::endl;

      //        srv_action.request.task = 1;


      if (callVisualPerceptors)
      {
        //	obj_center = srv_perception.response.pushable_object_center;
        //	obj_size   = srv_perception.response.pushable_object_size;
        srv_action.request.pushable_object_center = srv_perception.response.pushable_object_center;
        srv_action.request.pushable_object_size = srv_perception.response.pushable_object_size;
      }
      else
      {
        srv_action.request.pushable_object_center.resize (3);
        srv_action.request.pushable_object_size.resize (3);
        srv_action.request.pushable_object_center[0] = -0.35;
        srv_action.request.pushable_object_center[1] = 0.20;
        srv_action.request.pushable_object_center[2] = 0.15;
        srv_action.request.pushable_object_size[0] = 0.06;
        srv_action.request.pushable_object_size[1] = 0.06;
        srv_action.request.pushable_object_size[2] = 0.06;
      }

      ROS_INFO ( "calling action service");
      srv_action.request.experimentEpoch = experimentEpoch;
      ros::service::call ("/action", srv_action.request, srv_action.response);
      ROS_INFO ("finished action");
      yarp::os::Network::connect("/o:ReflexCommand", "/tactGraspRight/rpc:i");
      yarp::os::Network::connect("/icub/skin/righthandcomp", "/i:ReflexTactile");
      yarp::os::Network::connect("/o:ReflexCommand", "/tactGraspLeft/rpc:i");
      yarp::os::Network::connect("/icub/skin/lefthandcomp", "/i:ReflexTactile");

      //****************************************************

      exp_state_ = aff_msgs::ExperimentState::ASK_FOR_EFFECT;
      publishExpState ();
    }
    else if (exp_state_ == aff_msgs::ExperimentState::ASK_FOR_EFFECT)
    {
    
	//continue;

	int effect;
	
	std::cout << "What is the produced effect? (0: None, 1: Reached, 2: Push_Right, 3: Push_Left, 4: Push_Forward, 5: Pulled, 6: Disappeared, 7: Grasped, 8: Taken, 9: Given, 10: Rotated, 11: Dropped, {NEGATIVE}: Not thought about it yet)" << std::endl;
		
	std::cin >> effect;
	
	effect_asked_for = true;
	new_speech_cmd_rcvd_ = true;
      //      ROS_INFO ("*** ASK FOR EFFECT ! ***");
      
      /*
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
      }*/
      if (new_speech_cmd_rcvd_)
      {
        new_speech_cmd_rcvd_ = false;
        //if (aff_msgs::Speech::MAX_ACTION_INDEX < msg_speech_in.speech_cmd && msg_speech_in.speech_cmd
        //    < aff_msgs::Speech::MAX_EFFECT_INDEX)
        //{
          //effect speech command is received, now percept the environment
          effect_asked_for = false;//clear the flag

          //now call tuck arms and look at face to register face if any exists, and clear limbs from the table
          //TODO:
          //srv_action.request.experimentEpoch = experimentEpoch;
          /*
          if (srv_action.request.task == behavior_manager::Action::Request::TAKE)
          {
            //first drop the object
            ROS_INFO("dropping the object");
            
            srv_action.request.task = behavior_manager::Action::Request::RELEASE_DOWNWARD;
            srv_action.request.pushable_object_center = pushable_object_center_;
            srv_action.request.pushable_object_center[2] += 0.20;
            ros::service::call ("/action", srv_action.request, srv_action.response);
            ROS_INFO("finished -tuck_arms- action");
          }*/

          /*ROS_INFO("calling -tuck_arms- action service");
          srv_action.request.task = behavior_manager::Action::Request::TUCK_ARMS;
          ros::service::call ("/action", srv_action.request, srv_action.response);
          ROS_INFO("finished -tuck_arms- action");*/

          //          ros::Duration (5).sleep ();
          //          say(info_everything_ok);
          //          while(n->ok())
          //          {
          //            if(new_speech_cmd_rcvd_)
          //            {
          //              new_speech_cmd_rcvd_ = false;
          //              if(msg_speech_in.speech_cmd == aff_msgs::Speech::CONTINUE)
          //                break;
          //            }
          //          }
          std::cout << "*********************************************************" << std::endl;
          std::cout << "enter a key and press enter to continue extracting effect" << std::endl;
          std::cout << "*********************************************************" << std::endl;
          int i;
          cin >> i;

          exp_state_ = aff_msgs::ExperimentState::EFFECT;
          publishExpState ();
        //}
      }
    }
    else if (exp_state_ == aff_msgs::ExperimentState::EFFECT)
    {
      say (info_extracting);
      ROS_INFO ("*** EFFECT ! ***\n");
      //send behavior_manager a message so that robot goes for a configuration
      //which is required for the perception process (e.g. tuck_arms)

      //Assuming that this action is done successfully, now call perception service
	firstRun = true;
      if (callVisualPerceptors)
      {

      	//srv_perception.request.experimentEpoch = experimentEpoch;
        srv_perception.request.task = al_srvs::Perception::Request::EXTRACT_EFFECT;

        if (msg_speech_in.speech_cmd != aff_msgs::Speech::DISAPPEARED)
          srv_perception.request.arg = object_id_;//label of the object
        else
          srv_perception.request.arg = -1;//no such object, it is disappeared

        srv_perception.request.arg_effect = msg_speech_in.speech_cmd;

        ROS_INFO ("calling perception 3D service");
        ros::service::call ("/perception", srv_perception.request, srv_perception.response);
        ROS_INFO ("finished perception 3D");

        //finished 3d perception task, now do it with 2D sensors. But first, look at the object of
        //interest if it is still on the table
        //srv_perception_2D.request.experimentEpoch = experimentEpoch;
        srv_perception_2D.request.task = tabletop_2D_segmentation::Perception2D::Request::EXTRACT_EFFECT;
        if (msg_speech_in.speech_cmd != aff_msgs::Speech::DISAPPEARED)
        {
          //look at the object
          srv_action.request.experimentEpoch = experimentEpoch;
          srv_action.request.task = behavior_manager::Action::Request::LOOK_AT_POINT;
          std::cout << srv_perception.response.pushable_object_center[0] << " "
              << srv_perception.response.pushable_object_center[1] << " "
              << srv_perception.response.pushable_object_center[2] << std::endl;
          srv_action.request.pushable_object_center = srv_perception.response.pushable_object_center;

          ros::service::call ("/action", srv_action.request, srv_action.response);

          getObjectRegionIndices (srv_perception.response.pushable_object_center, region_row, region_col);
          srv_perception_2D.request.arg = (int8_t)region_row;
          srv_perception_2D.request.arg2 = (int8_t)region_col;
        }
        else
        {
          srv_perception_2D.request.arg = 0;
          srv_perception_2D.request.arg2 = -1;
        }

        ROS_INFO("calling perception 2D service");
        /*std::cout << (int)srv_perception_2D.request.arg << " " << (int)srv_perception_2D.request.arg2 << std::endl;
        ros::service::call (WORKSPACE_2D_SEGMENTATION_SRV_NAME, srv_perception_2D.request, srv_perception_2D.response);*/
      }
      action_asked_for_first = false;
      effect_asked_for_first = false;
      //      exp_state_ = aff_msgs::ExperimentState::ASK_FOR_ACTION;
      say (info_human);
      ROS_INFO("Now, human partner can make any action in the world! Then press a key and ENTER to continue!");
      exp_state_ = aff_msgs::ExperimentState::LET_HUMAN_ACT;
      publishExpState ();

      // ++Onur
      callVisualPerceptors = true;
      ROS_INFO("Visual perception callbacks are re-enabled");
      
      
      	// This is where the experiment epoch should normally increase. For testing purposes, 
      	incrementExperimentEpoch();
      // --Onur
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

  affordance_labels.resize (5);
  //  affordance_labels[REACHABLE]="reach-able";
  //  affordance_labels[TAKEABLE]="take-able";
  //  affordance_labels[GIVEABLE]="give-able";
  //  affordance_labels[GRASPABLE]="grasp-able";
  //  affordance_labels[VANISHABLE]="vanish-able";

  affordance_labels[0] = "reach-able";
  affordance_labels[1] = "take-able";
  affordance_labels[2] = "give-able";
  affordance_labels[3] = "grasp-able";
  affordance_labels[4] = "vanish-able";

  //ros initializations
  ros::init (argc, argv, "experiment_manager");
  init ();
  run ();
  fini ();

  return 0;
}
