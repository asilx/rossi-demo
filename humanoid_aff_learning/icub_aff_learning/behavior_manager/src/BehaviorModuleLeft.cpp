#include "BehaviorModule/BehaviorModule.h"
#include <iostream>
#include <fstream>
using namespace std;


BehaviorModule::BehaviorModule(ros::NodeHandle& n) {
  nh = n;
  srv_action = nh.advertiseService("/action",
				   &BehaviorModule::actionCallback, this);
}

BehaviorModule::~BehaviorModule() {

}

void BehaviorModule::push_right(Vector bb_center, Vector bb_dims, bool isUpper)
{
	choseArm(-1);

	if (bb_center[0] > -0.45)
	{
		evil();
		Vector reach_point = bb_center;
		Vector hand_orient = angleXZToVectorAngle(3 * PI / 2, PI);
		bool f;
    		//reach_point[2] += bb_dims[2]/2+0.08;
    		reach_point[1] -= (bb_dims[1]/2 + 0.05);
    		//reach_point[0] += 0.01;
    		reach_point[2] += 0.05;
    		//reach_point[0] += 0.03;
    		std::cout<<reach_point[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
    		action_left->pushAction(reach_point, hand_orient);
    		action_left->checkActionsDone(f, true);
    		//release(reach_point, false);
    		reach_point[1] += 0.08;
    		if (isUpper) reach_point[2] += bb_dims[2]/2 - 0.05;
    		else  reach_point[2] -= bb_dims[2]/2;
    		//release(reach_point, false);
    		action_left->pushAction(reach_point, hand_orient);
    		action_left->checkActionsDone(f, true);
    		reach_point[1] += 0.1;
    		action_left->disableContactDetection();
    		action_left->pushAction(reach_point, hand_orient);
    		action_left->checkActionsDone(f, true);
    		action_left->enableContactDetection();
    		happy();
	}


}

void BehaviorModule::push_left(Vector bb_center, Vector bb_dims, bool isUpper)
{
	choseArm(1);

	if (bb_center[0] > -0.45)
	{
		evil();
		Vector reach_point = bb_center;
		Vector hand_orient = angleXZToVectorAngle(3* PI / 2, PI);
		bool f;
    		//reach_point[2] += bb_dims[2]/2+0.08;
    		reach_point[1] += (bb_dims[1]/2 + 0.05);
    		//reach_point[0] += 0.01;
    		reach_point[2] += 0.05;
    		//reach_point[0] += 0.03;
    		std::cout<<reach_point[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
    		action_right->pushAction(reach_point, hand_orient);
    		action_right->checkActionsDone(f, true);
    		//release(reach_point, false);
    		reach_point[1] -= 0.08;
    		if (isUpper) reach_point[2] += bb_dims[2]/2 - 0.05;
    		else  reach_point[2] -= bb_dims[2]/2;
    		//release(reach_point, false);
    		action_right->pushAction(reach_point, hand_orient);
    		action_right->checkActionsDone(f, true);
    		reach_point[1] -= 0.1;
    		action_right->disableContactDetection();
    		action_right->pushAction(reach_point, hand_orient);
    		action_right->checkActionsDone(f, true);
    		action_right->enableContactDetection();
    		happy();
	}


}

void BehaviorModule::cover(Vector bb_center, Vector bb_dims) {
  //for right arm
  choseArm(bb_center[1]);
  if (bb_center[0] < -0.45) {
    std::cout<< "obj detected to be out of range" << std::endl;
    Vector reach_point = bb_center;
    reach_point[0] = -0.30; //+= bb_dims[0] / 2.0 + 0.09;//hand size
    reach_point[2] += bb_dims[2]/2 + 0.05;
    std::cout << "The limit: " << REACH_X_LIMIT << endl;
    std::cout << "Coordinates: " << bb_center[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
    release(reach_point, false);
  }
  else {
    Vector reach_point = bb_center;

    reach_point[2] += bb_dims[2]/2+0.08;
    reach_point[1] += 0.01;
    reach_point[0] += 0.03;
    std::cout<<reach_point[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
    release(reach_point, false);
    reach_point[2] -= 0.08;
    release(reach_point, false);
  }
}

void BehaviorModule::grasp(Vector bb_center, Vector bb_dims, bool isUpper)
{
  choseArm(bb_center[1]);
  if (bb_center[0] < -0.45) {
    std::cout<< "obj detected to be out of range" << std::endl;
    Vector reach_point = bb_center;
    reach_point[0] = -0.30; //+= bb_dims[0] / 2.0 + 0.09;//hand size
    reach_point[2] += bb_dims[2]/2 + 0.05;
    std::cout << "The limit: " << REACH_X_LIMIT << endl;
    std::cout << "Coordinates: " << bb_center[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
    release(reach_point, false);
  }
  else {
  	evil();
	Vector reach_point = bb_center;
	Vector hand_orient = angleXZToVectorAngle(3* PI / 2, PI);
	bool f;
    	//reach_point[2] += bb_dims[2]/2+0.08;
    	if(chosen_arm == "right") reach_point[1] += (bb_dims[1]/2 + 0.05);
    	else reach_point[1] -= (bb_dims[1]/2 + 0.05);
    	//reach_point[0] += 0.01;
    	reach_point[2] += 0.05;
    	reach_point[0] += 0.05;
    	std::cout<<reach_point[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
    	if(chosen_arm == "right") {
    		action_right->pushAction(reach_point, hand_orient);
    		action_right->checkActionsDone(f, true);
    	}
    	else
    	{
    		action_left->pushAction(reach_point, hand_orient);
    		action_left->checkActionsDone(f, true);
    	}
    	//release(reach_point, false);
    	if(chosen_arm == "right") reach_point[1] -= 0.08;
    	else reach_point[1] += 0.08;
    	if (isUpper) reach_point[2] += bb_dims[2]/2 - 0.05;
    	else  reach_point[2] -= bb_dims[2]/2;
    	//release(reach_point, false);
    	if(chosen_arm == "right") {
    		action_right->pushAction(reach_point, hand_orient);
    		action_right->checkActionsDone(f, true);
    	}
    	else
    	{
    		action_left->pushAction(reach_point, hand_orient);
    		action_left->checkActionsDone(f, true);
    	}

    	Bottle& btout_right = port_grasp_comm_right.prepare();
    	Bottle& btout_left = port_grasp_comm_left.prepare();
        btout_right.clear();
	btout_left.clear();
        // Do the closing action

        std::cout << "Closing the hand..." << std::endl;


        if(chosen_arm == "left")
        {
          std::string s = "gs";
          btout_left.addString(s.c_str());
          port_grasp_comm_left.write();
        }
        else
        {
          std::string s = "gs";
          btout_right.addString(s.c_str());
          port_grasp_comm_right.write();
        }
        Time::delay(3);
        reach_point[2] += 0.08;
        if(chosen_arm == "right") {
    		action_right->pushAction(reach_point, hand_orient);
    		action_right->checkActionsDone(f, true);
    	}
    	else
    	{
    		action_left->pushAction(reach_point, hand_orient);
    		action_left->checkActionsDone(f, true);
    	}

  }

}

void BehaviorModule::pull(Vector bb_center, Vector bb_dims) {
  //for right arm
  choseArm(bb_center[1]);
  if (bb_center[0] < -0.45) {
    std::cout<< "obj detected to be out of range" << std::endl;
    Vector reach_point = bb_center;
    reach_point[0] = -0.30; //+= bb_dims[0] / 2.0 + 0.09;//hand size
    reach_point[2] += bb_dims[2]/2 + 0.05;
    std::cout << "The limit: " << REACH_X_LIMIT << endl;
    std::cout << "Coordinates: " << bb_center[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
    release(reach_point, false);
  }
  else {
    Vector reach_point = bb_center;

    reach_point[2] += bb_dims[2]/2+0.08;
    if(chosen_arm == "right") reach_point[1] += 0.04;
    else reach_point[1] -= 0.04;
    reach_point[0] -= 0.04;
    std::cout<<reach_point[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
    release(reach_point, false);


    if(chosen_arm == "right") reach_point[1] -= 0.04;
    else reach_point[1] += 0.04;
    reach_point[2] -= 0.1;
    release(reach_point, false);

    Bottle& btout_right = port_grasp_comm_right.prepare();
    	Bottle& btout_left = port_grasp_comm_left.prepare();
        btout_right.clear();
	btout_left.clear();
        // Do the closing action

        std::cout << "Closing the hand..." << std::endl;


        if(chosen_arm == "left")
        {
          std::string s = "gs";
          btout_left.addString(s.c_str());
          port_grasp_comm_left.write();
        }
        else
        {
          std::string s = "gs";
          btout_right.addString(s.c_str());
          port_grasp_comm_right.write();
        }
    Time::delay(3);


    if(chosen_arm == "right") action_right->disableContactDetection();
    else action_left->disableContactDetection();

    reach_point[0] += 0.05;
    release(reach_point, false);

    if(chosen_arm == "right") action_right->enableContactDetection();
    else action_left->enableContactDetection();

    reach_point[2] += 0.08;
    release(reach_point, false);

    if (reach_point[2] <= -0.20) cout << " z limit exceeded " << endl;
  }
}

void BehaviorModule::reach(Vector bb_center, Vector bb_dims) {
  //for right arm
  choseArm(bb_center[1]);
  if (bb_center[0] < -0.45) {
    std::cout<< "obj detected to be out of range" << std::endl;
    Vector reach_point = bb_center;
    reach_point[0] = -0.30; //+= bb_dims[0] / 2.0 + 0.09;//hand size
    reach_point[2] += bb_dims[2]/2 + 0.05;
    std::cout << "The limit: " << REACH_X_LIMIT << endl;
    std::cout << "Coordinates: " << bb_center[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
    release(reach_point, false);
  }
  else {
    Vector reach_point = bb_center;

    reach_point[2] += bb_dims[2]/2+0.08;
    reach_point[1] += 0.01;
    reach_point[0] += 0.03;
    std::cout<<reach_point[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
    release(reach_point, false);
    reach_point[2] -= 0.08;
    release(reach_point, false);
    Bottle& btout_right = port_grasp_comm_right.prepare();
    	Bottle& btout_left = port_grasp_comm_left.prepare();
        btout_right.clear();
	btout_left.clear();
        // Do the closing action

        std::cout << "Closing the hand..." << std::endl;


        if(chosen_arm == "left")
        {
          std::string s = "gs";
          btout_left.addString(s.c_str());
          port_grasp_comm_left.write();
        }
        else
        {
          std::string s = "gs";
          btout_right.addString(s.c_str());
          port_grasp_comm_right.write();
        }
    Time::delay(3);
    reach_point[2] += 0.08;
    release(reach_point, false);

    if (reach_point[2] <= -0.20) cout << " z limit exceeded " << endl;
  }
}

void BehaviorModule::drop()
{
    Bottle& btout2 = port_grasp_comm_left.prepare();;
    if(chosen_arm == "left")
      btout2 = port_grasp_comm_left.prepare();
    else
      btout2 = port_grasp_comm_right.prepare();
    btout2.clear();


    std::string s2 = "oh";
    btout2.addString(s2.c_str());
    if(chosen_arm == "left")
      port_grasp_comm_left.write();
    else
      port_grasp_comm_right.write();
    tuckArms();
}


bool BehaviorModule::actionCallback(behavior_manager::Action::Request& request,
				    behavior_manager::Action::Response& response) {
  if (request.task != behavior_manager::Action::Request::DONT_ACT) {
    yarp::sig::Vector center(3);
    yarp::sig::Vector size(3);

    if (!(request.task == behavior_manager::Action::Request::HOME
	  || request.task == behavior_manager::Action::Request::TUCK_ARMS
	  || request.task
	  == behavior_manager::Action::Request::LOOK_AT_REGION
	  || request.task
	  == behavior_manager::Action::Request::LOOK_AT_FACE)) {

      bb_center_ = request.pushable_object_center;
      center[0] = bb_center_[0];
      center[1] = bb_center_[1];
      center[2] = bb_center_[2];// + 0.15;

      if (request.task
	  != behavior_manager::Action::Request::LOOK_AT_POINT) {
	bb_dims_ = request.pushable_object_size;
	size[0] = bb_dims_[0];
	size[1] = bb_dims_[1];
	size[2] = bb_dims_[2];
      }
    }
    bool logFlag = true;

    if (request.task == behavior_manager::Action::Request::HOME) {
      home();
      ROS_INFO("icub home");
      logFlag = false;
    }
    else if (request.task == behavior_manager::Action::Request::TUCK_ARMS) {
      ROS_INFO("tuck arms");
      tuckArms();
      logFlag = false;
    }
    else if (request.task == behavior_manager::Action::Request::LOOK_AT_POINT) {
      ROS_INFO("look at point");
      lookAtPoint(center);
      logFlag = false;
    }
    else if (request.task == behavior_manager::Action::Request::LOOK_AT_FACE) {
      ROS_INFO("look at face");
      lookAtFace();
      logFlag = false;
    }
    else if (request.task == behavior_manager::Action::Request::REACH) {
      ROS_INFO("reach");
      reach(center, size);
    }
    else if (request.task == behavior_manager::Action::Request::RELEASE) {
      ROS_INFO("release");
      release(center, false);
      logFlag = false;
    }
    else if (request.task == behavior_manager::Action::Request::PUSH_LEFT) {
         ROS_INFO("icub push left");
         push_left(center, size, false);
         openHand();
         tuckArms();
    } else if (request.task
     	       == behavior_manager::Action::Request::PUSH_RIGHT) {
       ROS_INFO("BehaviorModule:icub push right");
       push_right(center, size, false);
       openHand();
       tuckArms();

     }
     else if (request.task == behavior_manager::Action::Request::PUSH_LEFT_UPPER) {
         ROS_INFO("icub push left");
         push_left(center, size, true);
         openHand();
         tuckArms();
    } else if (request.task
     	       == behavior_manager::Action::Request::PUSH_RIGHT_UPPER) {
       ROS_INFO("BehaviorModule:icub push right");
       push_right(center, size, true);
       openHand();
       tuckArms();

     }
    else if (request.task
     	       == behavior_manager::Action::Request::COVER) {
       ROS_INFO("icub cover ");
       cover(center, size);
       //openHand();
       //tuckArms();

    }
    else if (request.task
     	       == behavior_manager::Action::Request::PUSH_FORWARD) {
       ROS_INFO("icub push forward");
       pull(center, size);
       openHand();
       tuckArms();

    }
    else if (request.task == behavior_manager::Action::Request::GRASP) {
         ROS_INFO("icub grasp");
         grasp(center, size, false);
         //openHand();
         //tuckArms();
    }
    else if (request.task == behavior_manager::Action::Request::GRASP_UPPER) {
         ROS_INFO("icub grasp");
         grasp(center, size, true);
         //openHand();
         //tuckArms();
    }
    else if (request.task == behavior_manager::Action::Request::DROP) {
         ROS_INFO("icub drop");
         drop();
    }

    // else if (request.task == behavior_manager::Action::Request::CLOSE_EYE_LIDS) {
    //   ROS_INFO("close eye lids");
    //   closeEyeLids();
    //   logFlag = false;
    // }
    // else if (request.task == behavior_manager::Action::Request::OPEN_EYE_LIDS) {
    //   ROS_INFO("open eye lids");
    //   openEyeLids();
    //   logFlag = false;
    // }
    // else if (request.task == behavior_manager::Action::Request::HAPPY) {
    //   ROS_INFO("happy");
    //   happy();
    //   logFlag = false;
    // }
    // else if (request.task == behavior_manager::Action::Request::ANGRY) {
    //   ROS_INFO("angry");
    //   angry();
    //   logFlag = false;
    // }
    // else if (request.task == behavior_manager::Action::Request::SAD) {
    //   ROS_INFO("sad");
    //   sad();
    //   logFlag = false;
    // }
    // else if (request.task == behavior_manager::Action::Request::EVIL) {
    //   ROS_INFO("evil");
    //   evil();
    //   logFlag = false;
    // }
    // else if (request.task == behavior_manager::Action::Request::NEUTRAL) {
    //   ROS_INFO("neutral");
    //   neutral();
    //   logFlag = false;
    // }

  }
  response.feedback = behavior_manager::Action::Response::DONE;
  return true;
}

void BehaviorModule::getArmDependentOptions(Bottle &b, Vector &_gOrien,
					    Vector &_gDisp, Vector &_dOffs, Vector &_dLift, Vector &_home_x) {
  if (Bottle *pB=b.find("grasp_orientation").asList()) {
    int sz = pB->size();
    int len = _gOrien.length();
    int l = len < sz ? len : sz;

    for (int i = 0; i < l; i++)
      _gOrien[i] = pB->get(i).asDouble();
  }

  if (Bottle *pB=b.find("grasp_displacement").asList()) {
    int sz = pB->size();
    int len = _gDisp.length();
    int l = len < sz ? len : sz;

    for (int i = 0; i < l; i++)
      _gDisp[i] = pB->get(i).asDouble();
  }

  if (Bottle *pB=b.find("systematic_error_displacement").asList()) {
    int sz = pB->size();
    int len = _dOffs.length();
    int l = len < sz ? len : sz;

    for (int i = 0; i < l; i++)
      _dOffs[i] = pB->get(i).asDouble();
  }

  if (Bottle *pB=b.find("lifting_displacement").asList()) {
    int sz = pB->size();
    int len = _dLift.length();
    int l = len < sz ? len : sz;

    for (int i = 0; i < l; i++)
      _dLift[i] = pB->get(i).asDouble();
  }

  if (Bottle *pB=b.find("home_position").asList()) {
    int sz = pB->size();
    int len = _home_x.length();
    int l = len < sz ? len : sz;

    for (int i = 0; i < l; i++)
      _home_x[i] = pB->get(i).asDouble();
  }
}

bool BehaviorModule::configure(ResourceFinder &rf) {

  robotName = rf.find("robot").asString().c_str();
  std::cout << robotName << " is the name of our robot" << std::endl;

  action_left = NULL;
  action_right = NULL;

  options_left.put("device", "remote_controlboard");
  options_left.put("local", "/pos_ctrl_left_arm");
  options_left.put("remote", ("/" + robotName + "/left_arm").c_str());

  options_right.put("device", "remote_controlboard");
  options_right.put("local", "/pos_ctrl_right_arm");
  options_right.put("remote", ("/" + robotName + "/right_arm").c_str());

  driver_left.open(options_left);
  driver_right.open(options_right);

  options_head.put("device", "remote_controlboard");
  options_head.put("local", "/pos_ctrl_head");
  options_head.put("remote", ("/" + robotName + "/head").c_str());

  driver_head.open(options_head);

  options_torso.put("device", "remote_controlboard");
  options_torso.put("local", "/pos_ctrl_torso");
  options_torso.put("remote", ("/" + robotName + "/torso").c_str());

  driver_torso.open(options_torso);

  emotP.open("/local/emoInt");
  Network::connect("/local/emoInt", "/icub/face/emotions/in");
  // become happy
  happy();

  if (!driver_left.isValid() || !driver_right.isValid() || !driver_head.isValid() || !driver_torso.isValid()) {
    cerr << "A device is not available. Here are the known devices:"
	 << endl;
    cerr << Drivers::factory().toString().c_str() << endl;
    exit(-1);
  }

  bool ok = true;
  ok = ok && driver_left.view(pos_ctrl_left);
  ok = ok && driver_left.view(encoders_left);
  ok = ok && driver_right.view(pos_ctrl_right);
  ok = ok && driver_right.view(encoders_right);
  ok = ok && driver_head.view(pos_ctrl_head);
  ok = ok && driver_head.view(encoders_head);
  ok = ok && driver_torso.view(pos_ctrl_torso);
  ok = ok && driver_torso.view(encoders_torso);
  ok = ok && driver_left.view(ictrl_left);
  ok = ok && driver_left.view(iimp_left);
  ok = ok && driver_left.view(itrq_left);
  ok = ok && driver_right.view(ictrl_right);
  ok = ok && driver_right.view(iimp_right);
  ok = ok && driver_right.view(itrq_right);
  ok = ok && driver_left.view(iamp_left);
  ok = ok && driver_left.view(ipid_left);
  ok = ok && driver_rigth.view(iamp_right);
  ok = ok && driver_right.view(ipid_right);


  chosen_arm = "left";
  // needed for impedance controller
  //ok = ok && driver_right.view(ictrl);
  //ok = ok && driver_right.view(trq_ctrl_left);
  //ok = ok && driver_right.view(trq_ctrl_right);

  options_gaze.put("device", "gazecontrollerclient");
  options_gaze.put("remote", "/iKinGazeCtrl");
  options_gaze.put("local", "/client/gaze");
  driver_gaze.open(options_gaze);
  driver_gaze.view(igaze);
  if (driver_gaze.isValid()) {
    ok = ok && driver_gaze.view(igaze);
  }
  igaze->setTrackingMode(false);
  igaze->setEyesTrajTime(1.0);
  igaze->setNeckTrajTime(2.0);
  igaze->bindNeckPitch(-38, 0);
  igaze->bindNeckRoll(-5, 5);
  igaze->bindNeckYaw(-30.0, 35.0);

  if (!ok) {
    cerr << "error getting interfaces" << std::endl;
    exit(-1);
  }

  int n_jnts = 0;

  pos_ctrl_left->getAxes(&n_jnts);
  positions_left_cmd.resize(n_jnts);
  positions_left_enc.resize(n_jnts);

  pos_ctrl_right->getAxes(&n_jnts);
  positions_right_cmd.resize(n_jnts);
  positions_right_enc.resize(n_jnts);

  pos_ctrl_head->getAxes(&n_jnts);
  positions_head_enc.resize(n_jnts);
  positions_head_cmd.resize(n_jnts);

  pos_ctrl_torso->getAxes(&n_jnts);
  positions_torso_enc.resize(n_jnts);
  positions_torso_cmd.resize(n_jnts);

  openPorts = false;
  firstRun = true;
  sim = false;

  string name = rf.find("name").asString().c_str();
  // setName(name.c_str());

  string sim_or_real = rf.find("sim").asString().c_str();
  if (sim_or_real == "on")
    sim = true;
  else
    sim = false;

  Property config;
  config.fromConfigFile(rf.findFile("from").c_str());
  Bottle &bGeneral = config.findGroup("general");
  if (bGeneral.isNull()) {
    cout << "Error: group general is missing!" << endl;
    return false;
  }

  // parsing general config options
  Property option;
  for (int i = 1; i < bGeneral.size(); i++) {
    Bottle *pB = bGeneral.get(i).asList();
    if (pB->size() == 2)
      option.put(pB->get(0).asString().c_str(), pB->get(1));
    else {
      cout << "Error: invalid option!" << endl;
      return false;
    }
  }

  option.put("local", name.c_str());
  option.put("grasp_model_type",
	     rf.find("grasp_model_type").asString().c_str());
  option.put("grasp_model_file", rf.findFile("grasp_model_file").c_str());
  option.put("hand_sequences_file",
	     rf.findFile("hand_sequences_file").c_str());

  printf("%s\n", option.toString().c_str());

  // parsing arm dependent config options
  Bottle &bArm = config.findGroup("arm_dependent");

  graspOrien.resize(4);
  graspDisp.resize(4);
  dOffs.resize(3);
  dLift.resize(3);
  home_x.resize(3);

  getArmDependentOptions(bArm, graspOrien, graspDisp, dOffs, dLift, home_x);

  option.put("part", "right_arm");
  printf("Options for right arm \n %s\n", option.toString().c_str());
  cout << "***** Instantiating primitives for right_arm" << endl;
  action_right = new ActionPrimitivesLayer2(option);
  option.put("part", "left_arm");
  printf("Options for left arm \n %s\n", option.toString().c_str());
  cout << "***** Instantiating primitives for left_arm"<< endl;
  action_left = new ActionPrimitivesLayer2(option);

  if (!action_left->isValid()) {
    delete action_left;
    cout << "Action_left is not valid" << endl;
    return false;
  }
  if (!action_right->isValid()) {
    delete action_right;
    cout << "Action_right is not valid" << endl;
    return false;
  }


  deque<string> q = action_right->getHandSeqList();
  cout << "***** List of available for right hand sequence keys:" << endl;
  for (size_t i = 0; i < q.size(); i++)
    cout << q[i] << endl;

  q = action_left->getHandSeqList();
  cout << "***** List of available for left hand sequence keys:" << endl;
  for (size_t i = 0; i < q.size(); i++)
    cout << q[i] << endl;
  return true;
}

bool BehaviorModule::close() {
  if (action_left != NULL)
    delete action_left;

  if (action_right != NULL)
    delete action_right;

  return true;
}

double BehaviorModule::getPeriod() {
  return 0.1;
}

void BehaviorModule::init() {
  port_grasp_comm_left.open("/o:graspCommLeft");
  port_grasp_comm_right.open("/o:graspCommRight");

  ROS_INFO("waiting for tactileGrasp module to be opened !");
  while (!Network::isConnected("/o:graspCommLeft", "/tactGraspLeft/rpc:i") && nh.ok()) {
    Network::connect("/o:graspCommLeft", "/tactGraspLeft/rpc:i");
    ros::spinOnce();
  }

  while (!Network::isConnected("/o:graspCommRight", "/tactGraspRight/rpc:i") && nh.ok()) {
    Network::connect("/o:graspCommRight", "/tactGraspRight/rpc:i");
    ros::spinOnce();
  }
  ROS_INFO("OK, connected to the tactGrasp modules! ");

  action_left->enableContactDetection();
  action_right->enableContactDetection();

}

Vector BehaviorModule::vectorAngle2Normal(Vector vec_angle_rep) {
  Matrix R = iCub::ctrl::axis2dcm(vec_angle_rep);

  Vector v(3);
  v[0] = R[0][2];
  v[1] = R[1][2];
  v[2] = R[2][2];
  return v;
}

Vector BehaviorModule::normal2VectorAngle(const Vector& hand_normal) {
  double z_theta_hand = atan2(hand_normal[1], hand_normal[0]);
  std::cout << "theta: " << z_theta_hand << std::endl;

  Vector ox(4);
  Vector oz_final(4);

  ox[0] = 1.0;
  ox[1] = 0.0;
  ox[2] = 0.0;
  ox[3] = -PI / 2;

  oz_final[0] = 0.0;
  oz_final[1] = 0.0;
  oz_final[2] = 1.0;
  oz_final[3] = z_theta_hand - PI / 2;

  Matrix Rx = iCub::ctrl::axis2dcm(ox);
  Matrix Rz_final = iCub::ctrl::axis2dcm(oz_final);

  Matrix R = Rz_final * Rx;
  Vector vec_ang = iCub::ctrl::dcm2axis(R);
  return vec_ang;
}

Vector BehaviorModule::angleXZToVectorAngle(const double x_ang,
					    const double z_ang) {
  Vector oz(4);
  Vector ox(4);

  oz[0] = 0.0;
  oz[1] = 0.0;
  oz[2] = 1.0;
  oz[3] = z_ang;// / 180 * PI;

  ox[0] = 1.0;
  ox[1] = 0.0;
  ox[2] = 0.0;
  ox[3] = x_ang;// / 180 * PI;

  Matrix Rz = iCub::ctrl::axis2dcm(oz); // from axis/angle to rotation matrix notation
  Matrix Rx = iCub::ctrl::axis2dcm(ox);

  Matrix R = Rz * Rx;
  Vector poi_orient = iCub::ctrl::dcm2axis(R); // from rotation matrix back to the axis/angle notation

  return poi_orient;
}

Vector BehaviorModule::angleXYZToVectorAngle(const double x_ang,
					     const double y_ang, const double z_ang) {
  Vector oz(4);
  Vector oy(4);
  Vector ox(4);

  oz[0] = 0.0;
  oz[1] = 0.0;
  oz[2] = 1.0;
  oz[3] = z_ang;// / 180 * PI;

  oy[0] = 0.0;
  oy[1] = 1.0;
  oy[2] = 0.0;
  oy[3] = y_ang;// / 180 * PI;

  ox[0] = 1.0;
  ox[1] = 0.0;
  ox[2] = 0.0;
  ox[3] = x_ang;// / 180 * PI;

  Matrix Rz = iCub::ctrl::axis2dcm(oz); // from axis/angle to rotation matrix notation
  Matrix Ry = iCub::ctrl::axis2dcm(oy);
  Matrix Rx = iCub::ctrl::axis2dcm(ox);

  Matrix R = Ry * Rz * Rx;
  Vector poi_orient = iCub::ctrl::dcm2axis(R); // from rotation matrix back to the axis/angle notation

  return poi_orient;
}


void BehaviorModule::release(Vector point, bool palm_upward) {
  Vector hand_orient;
  choseArm(point[1]);

  if(chosen_arm == "left")
    {
      if (palm_upward)
	hand_orient = angleXZToVectorAngle(PI, PI);
      else
	hand_orient = angleXZToVectorAngle(0, PI);
    }
  else
    {
      if (palm_upward)
	hand_orient = angleXZToVectorAngle(0, PI);
      else
	hand_orient = angleXZToVectorAngle(PI, PI);
    }

  bool f;
  std::cout<<"going to the location: "<<point[0]<<" "<<point[1]<<" "<<point[2]<<std::endl;

  if(chosen_arm == "left"){
    action_left->pushAction(point, hand_orient);
    action_left->checkActionsDone(f, true);
    cout << "Release action with left arm" << endl;
  }
  else{
    //for right arm, orientation should be reversed
    action_right->pushAction(point, hand_orient);
    action_right->checkActionsDone(f, true);
    cout << "Release action with right arm" << endl;
  }

}

void BehaviorModule::home(bool is_left_arm) {
  bool f, dn;
  Vector home_orient = angleXZToVectorAngle(-2 * PI / 5, PI);
  Vector js, home_coords;
  js.resize(positions_torso_enc.size());

  js[0] = 0;
  js[1] = 0;
  js[2] = 0;

  pos_ctrl_torso->positionMove(0, 0);

  dn = false;
  while (!dn)
    pos_ctrl_torso->checkMotionDone(0, &dn);

  pos_ctrl_torso->positionMove(js.data());

  dn = false;
  while (!dn)
    pos_ctrl_torso->checkMotionDone(&dn);

  home_coords = home_x;
  if (!is_left_arm)
    home_coords[1] *= -1;

  action_left->pushAction(home_coords, home_orient, "open_hand");
  action_right->pushAction(home_coords, home_orient, "open_hand");
  action_left->checkActionsDone(f, true);
  action_right->checkActionsDone(f, true);
}

void BehaviorModule::tuckArms() {

  action_left->pushAction("open_hand");
  action_right->pushAction("open_hand");
  ros::Duration(5).sleep();

  bool left_arm_cart_solver_active = false;
  bool right_arm_cart_solver_active = false;
  if (Network::isConnected(
			   "/actionPrimitivesMod/left_arm/position/command:o",
			   ("/" + robotName + "/cartesianController/left_arm/command:i").c_str()))
    left_arm_cart_solver_active = true;

  if (Network::isConnected(
			   "/actionPrimitivesMod/right_arm/position/command:o",
			   ("/" + robotName + "cartesianController/right_arm/command:i").c_str()))
    right_arm_cart_solver_active = true;

  if (left_arm_cart_solver_active) {
    Network::disconnect(
			"/actionPrimitivesMod/left_arm/position/command:o",
			("/" + robotName + "cartesianController/right_arm/command:i").c_str());
    Network::disconnect("/actionPrimitivesMod/left_arm/position/rpc:o",
			("/" + robotName + "left_arm/rpc:i").c_str());
    Network::disconnect(("/" + robotName + "left_arm/state:o").c_str(),
			"/actionPrimitivesMod/left_arm/position/state:i");
  }

  if (right_arm_cart_solver_active) {
    Network::disconnect(
			"/actionPrimitivesMod/right_arm/position/command:o",
			("/" + robotName + "cartesianController/right_arm/command:i").c_str());
    Network::disconnect("/actionPrimitivesMod/right_arm/position/rpc:o",
			("/" + robotName + "right_arm/rpc:i").c_str());
    Network::disconnect(("/" + robotName + "right_arm/state:o").c_str(),
			"/actionPrimitivesMod/right_arm/position/state:i");
  }

  bool dn;
  Vector js;
  js.resize(positions_torso_enc.size());

  js[0] = 0;
  js[1] = 0;
  js[2] = 0;

  pos_ctrl_torso->positionMove(2, 0);
  dn = false;
  while (!dn)
    pos_ctrl_torso->checkMotionDone(0, &dn);
  pos_ctrl_torso->positionMove(js.data());
  dn = false;
  while (!dn)
    pos_ctrl_torso->checkMotionDone(&dn);

  encoders_left->getEncoders(positions_left_enc.data());
  encoders_right->getEncoders(positions_right_enc.data());

  for (int i = 0; i < positions_left_enc.size(); i++) {
    pos_ctrl_left->setRefSpeed(i, 10.0);
    pos_ctrl_right->setRefSpeed(i, 10.0);
    pos_ctrl_left->setRefAcceleration(i, 50.0);
    pos_ctrl_right->setRefAcceleration(i, 50.0);
    //may change these values but damping and stiffnes should be adjusted accordingly for stability
    iimp_left->setImpedance(i, 0.111, 0.014);
    iimp_right->setImpedance(i, 0.111, 0.014);
  }

  //set command positions
  js.resize(positions_left_enc.size());

  bool wait;
  int control_mode_left;
  int control_mode_right;
  for (int i = 0; i < 4; ++i)
    {
      if (i == 3){
	ictrl_left->setTorqueMode(i);
	ictrl_right->setTorqueMode(i);
      }
      else
	{
	ictrl_left->setImpedancePositionMode(i);
	ictrl_right->setImpedancePositionMode(i);
	}
      ictrl_left->getControlMode(i, &control_mode_left);
      ictrl_right->getControlMode(i, &control_mode_right);
    }

  for (int i = 0; i < 4; ++i)
    {
      wait = true;
      while (wait){
	ictrl_left->getControlMode(i, &control_mode_left);
	ictrl_right->getControlMode(i, &control_mode_right);
	if (i ==3)
	  wait = !(control_mode_left == VOCAB_CM_TORQUE) && !(control_mode_right == VOCAB_CM_TORQUE);
	else
	  wait = !(control_mode_left == VOCAB_CM_IMPEDANCE_POS) && !(control_mode_right == VOCAB_CM_IMPEDANCE_POS);
	if(control_mode_left == VOCAB_CM_IDLE){
	  cout << "Control mode is idle for left arm joint " << i << endl;
	  iamp_left->enableAmp(i);
	  ipid_left->enablePid(i);
	  cout << "Enabled left arm joint " << i << endl;
	}
	if(control_mode_right == VOCAB_CM_IDLE){
	  cout << "Control mode is idle for right arm joint " << i << endl;
	  iamp_right->enableAmp(i);
	  ipid_right->enablePid(i);
	  cout << "Enabled right arm joint " << i << endl;
	}

	if (wait ==true)
	  cout << "Control mode wrong" << endl;
      }
    }

  js=0;
  js[0]=10;
  js[1]=20;
  js[2]=-30;
  js[3]=0;
  js[4]=-60;
  js[5]=0;
  js[6]=20;
  pos_ctrl_left->positionMove(js.data());
  pos_ctrl_right->positionMove(js.data());

  bool done=false;

  cout << "First set is done " << endl;
  Time::delay(5.0);

  cout << "Motion done " << endl;
  js[3]=70;
  ictrl_left->setImpedancePositionMode(3);
  ictrl_right->setImpedancePositionMode(3);
  wait = true;
  while (wait){
    ictrl_left->getControlMode(3, &control_mode_left);
    ictrl_right->getControlMode(3, &control_mode_right);
    wait = !(control_mode_left ==  VOCAB_CM_IMPEDANCE_POS) && !(control_mode_right ==  VOCAB_CM_IMPEDANCE_POS);;
    if (control_mode_left == VOCAB_CM_IDLE)
      {
	cout << "Left arm joint 3 is idle" << endl;
	iamp_left->enableAmp(joint);
	ipid_left->enablePid(joint);
      }
    if (control_mode_right == VOCAB_CM_IDLE)
      {
	cout << "Right arm joint 3 is idle" << endl;
	iamp_right->enableAmp(joint);
	ipid_right->enablePid(joint);
      }

    if (wait ==true)
      cout << "Control mode wrong for joint " << endl;
  }

  cout << "Second command " << endl;
  pos_ctrl_left->positionMove(js.data());
  pos_ctrl_right->positionMove(js.data());


  //finally connect cartesian solvers back
  if (left_arm_cart_solver_active) {
    //      driver_left.close();
    Network::connect(
		     "/actionPrimitivesMod/left_arm/position/command:o",
		     ("/" + robotName + "cartesianController/left_arm/command:i").c_str());
    Network::connect("/actionPrimitivesMod/left_arm/position/rpc:o", ("/"
								      + robotName + "left_arm/rpc:i").c_str());
    Network::connect(("/" + robotName + "left_arm/state:o").c_str(),
		     "/actionPrimitivesMod/left_arm/position/state:i");
  }

  if (right_arm_cart_solver_active) {
    //      driver_right.close();
    Network::connect(
		     "/actionPrimitivesMod/right_arm/position/command:o",
		     ("/" + robotName + "cartesianController/right_arm/command:i").c_str());
    Network::connect("/actionPrimitivesMod/right_arm/position/rpc:o", ("/"
								       + robotName + "right_arm/rpc:i").c_str());
    Network::connect(("/" + robotName + "right_arm/state:o").c_str(),
		     "/actionPrimitivesMod/right_arm/position/state:i");
  }
}

void BehaviorModule::lookAtPoint(Vector bb_center) {

  int i;
  std::cout << bb_center[0] << " " << bb_center[1] << " " << bb_center[2]
	    << std::endl;
  std::cout << "&&&&&& do you verify the position &&&&&&" << std::endl;

  igaze->lookAtFixationPoint(bb_center);
  bool done = false;
  while (!done) {
    done = igaze->waitMotionDone(0.2, 0.0);
  }
  sleep(3);
  std::cout << "lookAtPoint finished!" << std::endl;
}

void BehaviorModule::lookAtFace() {

  Vector face_center;
  face_center.resize(3);
  face_center[0] = -2.0;
  face_center[1] = 0.0;
  face_center[2] = 0.4;

  igaze->lookAtFixationPoint(face_center);
  bool done = false;
  while (!done) {
    done = igaze->waitMotionDone(0.2, 0.0);
  }
  sleep(3);
  std::cout << "lookAtPoint finished!" << std::endl;
}

void BehaviorModule::choseArm(double y_position){

  chosen_arm = "left";
  if (y_position <0.0){
    chosen_arm = "left";
    cout << "Left arm is chosen" << endl;
  }
  if (y_position >0.0){
    cout << "Right arm is chosen" << endl;
    chosen_arm = "right";
  }
}

void BehaviorModule::testHandSequences() {
  bool f;
  cout << "Starting testHandSequences" << endl ;
  cout << "Checking Left Hand" << endl ;
  action_left->pushAction("close_hand");
  action_left->checkActionsDone(f, true);
  action_left->areFingersInPosition(f);

  action_left->pushAction("open_hand");
  action_left->checkActionsDone(f, true);
  action_left->areFingersInPosition(f);

  action_left->pushAction("close_hand");
  action_left->checkActionsDone(f, true);
  action_left->areFingersInPosition(f);

  action_left->pushAction("karate_hand");
  action_left->checkActionsDone(f, true);
  action_left->areFingersInPosition(f);
  cout << "End of left hand check" << endl ;
  cout << "Checking Right Hand" << endl ;
  action_right->pushAction("close_hand");
  action_right->checkActionsDone(f, true);
  action_right->areFingersInPosition(f);

  action_right->pushAction("open_hand");
  action_right->checkActionsDone(f, true);
  action_right->areFingersInPosition(f);

  action_right->pushAction("close_hand");
  action_right->checkActionsDone(f, true);
  action_right->areFingersInPosition(f);

  action_right->pushAction("karate_hand");
  action_right->checkActionsDone(f, true);
  action_right->areFingersInPosition(f);
  cout << "End of right hand check" << endl ;

}

void BehaviorModule::openHand()
{
  std::cout << "Done. Now, opening the hand..." << std::endl;
  std::string s;
  Bottle& btout2 = port_grasp_comm_left.prepare();;
  if (chosen_arm == "left")
    btout2 = port_grasp_comm_left.prepare();
  else
    btout2 = port_grasp_comm_right.prepare();
  btout2.clear();
  s = "oh";
  btout2.addString(s.c_str());
  if (chosen_arm == "left")
    port_grasp_comm_left.write();
  else
    port_grasp_comm_right.write();
  std::cout << "Done. Have a nice day!" << std::endl;

}

// we don't need a thread since the actions library already
// incapsulates one inside dealing with all the tight time constraints
bool BehaviorModule::updateModule() {

  // do it only once
  if (firstRun) {
    init();
    firstRun = false;
  }

  Vector bb_dims(3);
  bb_dims[0] = 0.06;
  bb_dims[1] = 0.06;
  bb_dims[2] = 0.06;

  // get a target object position from a YARP port

  Vector bb_center(3);

  bb_center[0] = -0.30;
  bb_center[1] = 0.15;
  bb_center[2] = 0.20;

  bool contactDetected = false;
  double totalReading = 0.0;

  ros::spinOnce();
  return true;
}

bool BehaviorModule::interruptModule() {
  // since a call to checkActionsDone() blocks
  // the execution until it's done, we need to
  // take control and exit from the waiting state
  action_left->syncCheckInterrupt(true);
  action_right->syncCheckInterrupt(true);


  return true;
}

void BehaviorModule::openEyeLids()
{
  outBot.clear();
  outBot.addString("set");
  outBot.addString("eli");
  outBot.addString("ang");

  emotP.write(outBot);
}

void BehaviorModule::closeEyeLids()
{
  outBot.clear();
  outBot.addString("set");
  outBot.addString("raw");
  outBot.addString("S04");
  emotP.write(outBot);
}

void BehaviorModule::happy()
{
  outBot.clear();
  outBot.addString("set");
  outBot.addString("all");
  outBot.addString("hap");
  emotP.write(outBot);
}

void BehaviorModule::angry()
{
  outBot.clear();
  outBot.addString("set");
  outBot.addString("all");
  outBot.addString("ang");
  emotP.write(outBot);
}

void BehaviorModule::sad()
{
  outBot.clear();
  outBot.addString("set");
  outBot.addString("all");
  outBot.addString("sad");
  emotP.write(outBot);
}

void BehaviorModule::evil()
{
  outBot.clear();
  outBot.addString("set");
  outBot.addString("all");
  outBot.addString("evi");
  emotP.write(outBot);
}

void BehaviorModule::neutral()
{
  outBot.clear();
  outBot.addString("set");
  outBot.addString("all");
  outBot.addString("neu");
  emotP.write(outBot);
}
