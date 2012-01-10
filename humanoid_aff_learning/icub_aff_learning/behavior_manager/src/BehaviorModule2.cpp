#include "BehaviorModule/BehaviorModule.h"

// ++Onur: Data logging interface functions here!

// Get the end-effector position from the action primitive layer object
void BehaviorModule::getEEPosition() {
	Vector pos, orient;

	action->getPose(pos, orient);

	armPoseInfo.features[0] = pos[0]; // x;
	armPoseInfo.features[1] = pos[2]; // y;
	armPoseInfo.features[2] = pos[1]; // z;		
	armPoseInfo.features[3] = orient[0]; //xa;
	armPoseInfo.features[4] = orient[1]; //ya;
	armPoseInfo.features[5] = orient[2]; //za;
	armPoseInfo.features[6] = orient[3]; //theta;

}

// Log the given data (currently, the armPoseInfo:FeatureTuple)
void BehaviorModule::logPosData(uint8_t task_id) {
	getEEPosition();
	getHeadJoints();

	armLogger->logSingleData(&armPoseInfo, (int) task_id);
	headLogger->logSingleData(&headPoseInfo, (int) task_id);

}



void BehaviorModule::getHeadJoints() {
	encoders_head->getEncoders(positions_head_enc.data());
	encoders_torso->getEncoders(positions_torso_enc.data());

	headPoseInfo.features[0] = positions_head_enc[0]; // joint 0
	headPoseInfo.features[1] = positions_head_enc[1]; // joint 1
	headPoseInfo.features[2] = positions_head_enc[2]; // etc
	headPoseInfo.features[3] = positions_head_enc[3];
	headPoseInfo.features[4] = positions_head_enc[4];
	headPoseInfo.features[5] = positions_head_enc[5];

	headPoseInfo.features[6] = positions_torso_enc[0];
	headPoseInfo.features[7] = positions_torso_enc[1];
	headPoseInfo.features[8] = positions_torso_enc[2];

}

// --Onur
BehaviorModule::BehaviorModule(ros::NodeHandle& n) {

	nh = n;
	srv_action = nh.advertiseService("/action",
			&BehaviorModule::actionCallback, this);

	pub_internal_features_ = nh.advertise<aff_msgs::Features>("/internal_features", 10);
}

BehaviorModule::~BehaviorModule() {

	// ++Onur
	delete armLogger;
	delete headLogger;
	delete tactileLogger;
	delete[] armPoseInfo.features; ///++kadir: 
	delete[] headPoseInfo.features;
	delete[] tactileInfo.features;
	// --Onur

}

void BehaviorModule::take() {
	Vector center(3);	center[0] = -0.30; center[1] = 0.20; center[2] = 0.10;
	releaseUpward(center);

	// if any tactile information is obtained, close the hand to grasp the object.
	// wait for 5 seconds for this to occur

	Bottle& btout = port_grasp_comm.prepare();
	btout.clear();
	
	//now close hand
	//std::cout<<" ooooohhhhhh called "<<std::endl;
	//std::string s = "oh";
	//btout.addString(s.c_str());
	//port_grasp_comm.write();
	//ros::Duration(3).sleep();//wait here for three second
	
	//std::cout<<" open hand called "<<std::endl;
	
	/*
	bool f;
	action->pushAction("open_hand");
	action->checkActionsDone(f, true);
	
	ros::Duration(3).sleep();//wait here for three second
	*/
	ros::Time t1 = ros::Time::now();
	bool hand_closed = false;

	//if there is no object given in 5 seconds, then close the hand
	while ((ros::Time::now() - t1).toSec() < 5 && nh.ok() && !hand_closed) {
		Bottle *b = tactileReader_in.read();
		bool sth_in_palm = false;
		// now, b is supposed to contain 48 doubles of tactile information. fill this to tactileInfo.
		std::cout << "The tactile data was read as follows: " << std::endl;
	 for (int i = 8; i < 12; i++) {
	 	for (int j=0;j<12;j++)
	 		std::cout<< b->get(i*12+j).asDouble()<<" ";
	 	std::cout<<std::endl;
		}
		std::cout<<" are you done lookin at tactile data, then press a key and enter"<<std::endl;
		int k;
		//cin>>k;

		//ros::Duration(4).sleep(); // TODO: a different mechanism here.
//		for (int i = 0; i < 48; i++) {
		for (int i = 96; i < 144; i++) {
			if ((255.0 - b->get(i).asDouble()) > 15)//tactile sensing threshold
			{
				sth_in_palm = true;
				break;
			}
		}
		
		if(sth_in_palm)
			std::cout<<"***** I got something! Press a key and enter to close the hand*****"<<std::endl;
		else
		{
			// Otherwise, skip the remaining procedures. If  
			std::cout<<"***** I got nothing! Waiting...*****"<<std::endl;
			continue;
		}
		int i;
		//cin>>i;
		
		Bottle& btout2 = port_grasp_comm.prepare();
		btout2.clear();
	
		std::string s = "gs";
		btout2.addString(s.c_str());
		port_grasp_comm.write();
		hand_closed = true;
		ros::Duration(3).sleep();//wait here for three second
		
		//now close hand
//		btout.clear();
//		std::string s = "gs";
//		btout.addString(s.c_str());
//		port_grasp_comm.write();
//		hand_closed = true;
//		ros::Duration(3).sleep();//wait here for three second
	}

	if(!hand_closed)
	{
		std::cout << "I did not close the hand. (Got out of the loop before doing so)" << std::endl;
	}
}


void
BehaviorModule::drop(Vector point)
{
	Vector hand_orient = angleXZToVectorAngle(PI, PI);

	point[0]=-0.30;
	point[1]= 0.10;
	point[2]= 0.15;

	bool f;
	action->pushAction(point, hand_orient);
	action->checkActionsDone(f, true);
	
	Bottle& btout = port_grasp_comm.prepare();
	btout.clear();
	
	//now close hand
	std::string s = "oh";
	btout.addString(s.c_str());
	port_grasp_comm.write();
	ros::Duration(5).sleep();//wait here for three second
}

void BehaviorModule::give() {

	Vector center(3);
	center[0] = -0.40; center[1] = 0.15; center[2] = 0.15;

	//release(center, false);
  releaseUpward(center);
	
	Bottle& btout = port_grasp_comm.prepare();
	btout.clear();
	
	//now open hand
	std::string s = "oh";
	btout.addString(s.c_str());
	port_grasp_comm.write();
	ros::Duration(3).sleep();//wait here for three second
	/*
	bool f;
	action->pushAction("open_hand");
	action->checkActionsDone(f, true);
	ros::Duration(3).sleep();//wait here for three second	
	*/
}

void BehaviorModule::giveAfterTake() {
	take();//first do take action to obtain an object so that robot can give it later
	logTactileData();
	int i;
	std::cout << "Please take what I am holding" << std::endl;
	
	// TODO: tactile feedback here
	// cin >> i;
	give();
}

void BehaviorModule::reach(Vector bb_center, Vector bb_dims) {
	//for right arm
	if (bb_center[0] < REACH_X_LIMIT) {
		std::cout<< "obj detected to be out of range" << std::endl;
		Vector reach_point = bb_center;
		reach_point[0] = -0.30; //+= bb_dims[0] / 2.0 + 0.09;//hand size
		reach_point[2] += bb_dims[2]/2 + 0.05;
		std::cout<<reach_point[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
		int i;
		//cin>>i;
		release(reach_point, false);
	} else {
		Vector reach_point = bb_center;
		
		reach_point[2] += bb_dims[2]/2+0.05;
		std::cout<<reach_point[0]<<" "<<reach_point[1]<<" "<<reach_point[2]<<std::endl;
		int i;
		//cin>>i;
		release(reach_point, false);
	}
}

void BehaviorModule::logTactileData() {
	Bottle *b = tactileReader_in.read();

	// now, b is supposed to contain 48 doubles of tactile information. fill this to tactileInfo.

	for (int i = 0; i < 48; i++) {
		tactileInfo.features[i % 4] += 255.0 - b->get(i).asDouble();
	}


	for (int i = 96; i < 144; i++)
	{
		tactileInfo.features[4 + i%4] += 255.0 - b->get(i).asDouble();
	}
	
	for (int i = 0; i < 8; i++) {
		tactileInfo.features[i] = tactileInfo.features[i] / 12;
	}

	tactileLogger->logSingleData(&tactileInfo, 0);
}

bool BehaviorModule::actionCallback(behavior_manager::Action::Request& request,
		behavior_manager::Action::Response& response) {
		
		
	while(true)
	{
		Bottle *b = tactileReader_in.read();
		bool sth_in_palm = false;
		// now, b is supposed to contain 48 doubles of tactile information. fill this to tactileInfo.
		std::cout << "The tactile data was read as follows: " << std::endl;
		for (int i = 8; i < 12; i++) 
		{
			for (int j=0;j<12;j++)
				std::cout<< b->get(i*12+j).asDouble()<<" ";
			std::cout<<std::endl;
		}
	
		ros::Duration(1).sleep();	
	
	}	
		
		
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

			//			center[0] = -0.30;
			//			center[1] = -0.20;
			//			center[2] = 0.10;
			//			size[0] = 0.06;
			//			size[1] = 0.06;
			//			size[2] = 0.06;
			//			std::cout << request.pushable_object_center[0] << " "
			//					<< request.pushable_object_center[1] << " "
			//					<< request.pushable_object_center[2] << std::endl;
			//
			//			std::cout << request.pushable_object_size[0] << " "
			//					<< request.pushable_object_size[1] << " "
			//					<< request.pushable_object_size[2] << std::endl;
		}

		// ++Onur: log the initial pose configurationsBufferedPort<Bottle> port_grasp_out;
		//logPosData(request.task);
		//logTactileData();
		// --Onur
		
		cout << "Obj. coordinates; please confirm:" << endl;
		cout << center[0] << "   " << center[1] << "   " << center[2] << endl;
		int x;
		
		cin >> x;
		bool logFlag = true;
		bool goToHomeFlag = false;

		if (request.task == behavior_manager::Action::Request::PUSH_LEFT) {
			
			ROS_INFO("icub push left");
			push2(center, size, PI, 0.18, false, false);
			tuckArms();
			goToHomeFlag = true;
			ros::Duration(2).sleep();
			
		} else if (request.task
				== behavior_manager::Action::Request::PUSH_RIGHT) {
			ROS_INFO("BehaviorModule:icub push right");
			push2(center, size, 0, 0.15, false, false);
			goToHomeFlag = true;
		} else if (request.task
				== behavior_manager::Action::Request::PUSH_FORWARD) {
			ROS_INFO("icub push forward");
			push2(center, size, PI / 2, 0.15, false, false);
			goToHomeFlag = true;
		} else if (request.task
				== behavior_manager::Action::Request::PUSH_BACKWARD) {
			//			push(center, size, -PI / 2, 0.15, false);
			ROS_INFO("icub push backward");
			goToHomeFlag = true;
		} else if (request.task == behavior_manager::Action::Request::HOME) {
			home();
			ROS_INFO("icub home");
			logFlag = false;
		} else if (request.task == behavior_manager::Action::Request::TUCK_ARMS) {
			ROS_INFO("tuck arms");
			tuckArms();
			logFlag = false;
		} else if (request.task
				== behavior_manager::Action::Request::LOOK_AT_REGION) {
			lookAtRegion(request.arg);
			ROS_INFO("look at region");
			logFlag = false;
		} else if (request.task
				== behavior_manager::Action::Request::LOOK_AT_POINT) {
			ROS_INFO("look at point");
			lookAtPoint(center);
			logFlag = false;
		} else if (request.task
				== behavior_manager::Action::Request::LOOK_AT_FACE) {
			ROS_INFO("look at face");
			lookAtFace();
			logFlag = false;
		} else if (request.task == behavior_manager::Action::Request::GRASP) {
			ROS_INFO("grasp");
			Vector upward(3);
			upward[0] = center[0];
			upward[1] = center[1];
			upward[2] = center[2] + size[2] / 2;
			grasp2(upward);
		} else if (request.task == behavior_manager::Action::Request::REACH) {
			ROS_INFO("reach");
			reach(center, size);
			goToHomeFlag = true;
		} else if (request.task == behavior_manager::Action::Request::TAKE) {
			ROS_INFO("take");
			take();
			goToHomeFlag = true;
		} else if (request.task == behavior_manager::Action::Request::GIVE) {
			ROS_INFO("give");
			giveAfterTake();
			goToHomeFlag = true;
		} else if (request.task == behavior_manager::Action::Request::RELEASE) {
			ROS_INFO("release");
			release(center, false);
			logFlag = false;
		} else if (request.task == behavior_manager::Action::Request::RELEASE_UPWARD) {
			ROS_INFO("release upward");
			releaseUpward(center);
			logFlag = false;
		} else if (request.task == behavior_manager::Action::Request::RELEASE_DOWNWARD) {
			ROS_INFO("release downward");
//			releaseDownward(center);
			drop(center);
			logFlag = false;
		}

		// ++Onur: log the final pose configurations
		if (logFlag) {
			logTactileData();
			logPosData(request.task);
		}
		// --Onur

//		if (goToHomeFlag)//request.task != behavior_manager::Action::Request::HOME	|| request.task != behavior_manager::Action::Request::GRASP) {
//		{
//			home();
//			tuckArms();
//		}

	}
	response.feedback = behavior_manager::Action::Response::DONE;
	return true;
}

void BehaviorModule::grasp2(Vector bb_upward) {

	Bottle& bot = port_grasp_out.prepare();
	bot.clear();
	double lim = -0.45;
	cout<<"Will enter according to limit"<<endl;
	cout<<"REACH_X_LIMIT"<<lim<<endl;
	cout<<"bb_upward(0)"<<bb_upward(0)<<endl;
	if (bb_upward(0) > lim) {
		int x;
		std::cout << "Sending coordinates w open hand, please confirm:\n"
				<< bb_upward(0) << " " << bb_upward(1) << " " << (bb_upward(2)
				+ 0.02) << std::endl;
		//std::cin >> x;
		bot.addDouble(bb_upward(0));
		bot.addDouble(bb_upward(1));
		bot.addDouble(bb_upward(2) + 0.04);
		bot.addInt(1);
		port_grasp_out.write();
		ros::Duration(2).sleep();

		bot = port_grasp_out.prepare();
		bot.clear();

		std::cout << "Sending coordinates w close hand, please confirm:\n"
				<< (bb_upward(0) - 0.03) << " " << bb_upward(1) << " "
				<< (bb_upward(2) - 0.01) << std::endl;
		//std::cin >> x;
		bot.addDouble(bb_upward(0));
		bot.addDouble(bb_upward(1));
		bot.addDouble(bb_upward(2) - 0.01);
		bot.addInt(0);
		port_grasp_out.write();
		ros::Duration(2).sleep();

		bot = port_grasp_out.prepare();
		bot.clear();
		std::cout << "Sending coordinates, please confirm:\n" << bb_upward(0)
				<< " " << bb_upward(1) << " " << (bb_upward(2) + 0.07) << std::endl;
		//std::cin >> x;
		bot.addDouble(bb_upward(0));
		bot.addDouble(bb_upward(1));
		bot.addDouble(bb_upward(2) + 0.07);
		bot.addInt(1);
		port_grasp_out.write();

		ros::Duration(2).sleep();
	}
	else
	{
		cout<<"Object out of range!"<<endl;
	}

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
	//	robotName = "icub";

	graspOrien.resize(4);
	graspDisp.resize(4);
	dOffs.resize(3);
	dLift.resize(3);
	home_x.resize(3);

	// default values for arm-dependent quantities
	graspOrien[0] = -0.171542;
	graspOrien[1] = 0.124396;
	graspOrien[2] = -0.977292;
	graspOrien[3] = 3.058211;

	graspDisp[0] = 0.0;
	graspDisp[1] = 0.0;
	graspDisp[2] = 0.05;

	dOffs[0] = -0.03;
	dOffs[1] = -0.07;
	dOffs[2] = -0.02;

	dLift[0] = 0.0;
	dLift[1] = 0.0;
	dLift[2] = 0.15;

	home_x[0] = -0.29;
	home_x[1] = -0.21;
	home_x[2] = 0.11;

	action = NULL;

	openPorts = false;
	firstRun = true;
	sim = false;

	options_left.put("device", "remote_controlboard");
	options_left.put("local", "/pos_ctrl_left_arm");
	options_left.put("remote", ("/" + robotName + "/left_arm").c_str());

	options_right.put("device", "remote_controlboard");
	options_right.put("local", "/pos_ctrl_right_arm");
	options_right.put("remote", ("/" + robotName + "/right_arm").c_str());

	//driver_left.open(options_left);
	driver_right.open(options_right);

	// ++Onur: Connection to head port...

	options_head.put("device", "remote_controlboard");
	options_head.put("local", "/pos_ctrl_head");
	options_head.put("remote", ("/" + robotName + "/head").c_str());

	driver_head.open(options_head);

	options_torso.put("device", "remote_controlboard");
	options_torso.put("local", "/pos_ctrl_torso");
	options_torso.put("remote", ("/" + robotName + "/torso").c_str());

	driver_torso.open(options_torso);

	// --Onur


	if (/*!driver_left.isValid() || */!driver_right.isValid()
			|| !driver_head.isValid() || !driver_torso.isValid()) {
		cerr << "A device is not available. Here are the known devices:"
				<< endl;
		cerr << Drivers::factory().toString().c_str() << endl;
		exit(-1);
	}

	bool ok = true;
	//ok = ok && driver_left.view(pos_ctrl_left);
	ok = ok && driver_right.view(pos_ctrl_right);
	//ok = ok && driver_left.view(encoders_left);
	ok = ok && driver_right.view(encoders_right);

	//++onur

	ok = ok && driver_head.view(pos_ctrl_head);
	ok = ok && driver_head.view(encoders_head);
	ok = ok && driver_torso.view(pos_ctrl_torso);
	ok = ok && driver_torso.view(encoders_torso);

	//--onur

	options_gaze.put("device", "gazecontrollerclient");
	options_gaze.put("remote", "/iKinGazeCtrl");
	options_gaze.put("local", "/client/gaze");
	driver_gaze.open(options_gaze);

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

	/*
	 pos_ctrl_left->getAxes(&n_jnts);
	 positions_left_cmd.resize(n_jnts);
	 positions_left_enc.resize(n_jnts);
	 */

	pos_ctrl_right->getAxes(&n_jnts);
	positions_right_cmd.resize(n_jnts);
	positions_right_enc.resize(n_jnts);

	// ++Onur
	pos_ctrl_head->getAxes(&n_jnts);
	positions_head_enc.resize(n_jnts);
	positions_head_cmd.resize(n_jnts);

	pos_ctrl_torso->getAxes(&n_jnts);
	positions_torso_enc.resize(n_jnts);
	positions_torso_cmd.resize(n_jnts);

	// --Onur

	//  driver_left.close();
	//  driver_right.close();

	string name = rf.find("name").asString().c_str();
	setName(name.c_str());

	string sim_or_real = rf.find("sim").asString().c_str();
	if (sim_or_real == "on")
		sim = true;
	else
		sim = false;

	string partUsed = rf.find("part").asString().c_str();
	if ((partUsed != "left_arm") && (partUsed != "right_arm")) {
		cout << "Invalid part requested!" << endl;
		return false;
	}

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
	option.put("part", rf.find("part").asString().c_str());
	option.put("grasp_model_type",
			rf.find("grasp_model_type").asString().c_str());
	option.put("grasp_model_file", rf.findFile("grasp_model_file").c_str());
	option.put("hand_sequences_file",
			rf.findFile("hand_sequences_file").c_str());

	printf("%s\n", option.toString().c_str());

	// parsing arm dependent config options
	Bottle &bArm = config.findGroup("arm_dependent");
	getArmDependentOptions(bArm, graspOrien, graspDisp, dOffs, dLift, home_x);

	cout << "***** Instantiating primitives for " << partUsed << endl;
	action = new ActionPrimitivesLayer2(option);
	if (!action->isValid()) {
		delete action;
		return false;
	}

	deque<string> q = action->getHandSeqList();
	cout << "***** List of available hand sequence keys:" << endl;
	for (size_t i = 0; i < q.size(); i++)
		cout << q[i] << endl;

	string fwslash = "/";
	inPort.open((fwslash + name + "/in").c_str());
	rpcPort.open((fwslash + name + "/rpc").c_str());
	port_simon_in.open("/simon_cmd:i");
	port_grasp_out.open("/grasp_behavior:o");
	tactileReader_in.open("/i:tactileListener");
	attach(rpcPort);

	//string tactilePort = "/icub/skin/" + partUsed + "hand";
	yarp::os::Network::connect("/grasp_behavior:o", "/i:grasper");

	if (partUsed == "right_arm") {
		yarp::os::Network::connect("/icub/skin/righthand", "/i:tactileListener");
	} else {
		yarp::os::Network::connect("/icub/skin/lefthand", "/i:tactileListener");
	}
	openPorts = true;

	// check whether the grasp model is calibrated,
	// otherwise calibrate it and save the results
	Model *model;
	action->getGraspModel(model);
	if (!model->isCalibrated()) {
		Property prop("(finger all)");
		model->calibrate(prop);

		ofstream fout;
		fout.open(option.find("grasp_model_file").asString().c_str());
		model->toStream(fout);
		fout.close();
	}

	// ++Onur: logging information

	// TODO: should extend the headLogger's feature count after the torso information is also received
	armLogger = new DataLogger(partUsed.c_str(), "./", 7);
	headLogger = new DataLogger("head", "./", 9);

	string tact = "tactile_" + partUsed;
	tactileLogger = new DataLogger(tact.c_str(), "./", 8);
	armPoseInfo.features = new double[7];
	headPoseInfo.features = new double[9];
	tactileInfo.features = new double[8];
	// --Onur

	return true;
}

bool BehaviorModule::close() {
	if (action != NULL)
		delete action;

	if (openPorts) {
		inPort.close();
		rpcPort.close();
		driver_gaze.close();
	}

	return true;
}

double BehaviorModule::getPeriod() {
	return 0.1;
}

void BehaviorModule::init() {
	//	bool f;

	if (sim) {
		port_sim_rpc_out.open("/sim:o");
		Network::connect("/sim:o", "/icubSim/world");
	}

	port_grasp_comm.open("/o:graspComm");
	ROS_INFO("waiting for tactileGrasp module to be opened !");
	/*while (!Network::isConnected("/o:graspComm", "/tactGrasp/rpc:i") && nh.ok()) {
		Network::connect("/o:graspComm", "/tactGrasp/rpc:i");
		ros::spinOnce();
	}*/

	action->disableContactDetection();
	//	action->pushAction(home_x, "open_hand");
	//	action->checkActionsDone(f, true);
	//	action->enableContactDetection();
	//  action->enableArmWaving (home_x);
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
	//    Vector oz (4);
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

	//    Matrix Rz = iCub::ctrl::axis2dcm (oz); // from axis/angle to rotation matrix notation
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

	//    std::cout << poi_orient[0] << " " << poi_orient[1] << " " << poi_orient[2] << std::endl;

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
	//    Matrix R = Rz * Rx;
	Vector poi_orient = iCub::ctrl::dcm2axis(R); // from rotation matrix back to the axis/angle notation

	//    std::cout << poi_orient[0] << " " << poi_orient[1] << " " << poi_orient[2] << std::endl;

	return poi_orient;
}

//bb_pos and bb_dims specifies the bounding box properties of the object of interest
//push_dir is the angle(in radians). 0 -> right, PI/2 -> forward, PI -> left, 3*PI/2 -> backward
void BehaviorModule::push(const Vector& bb_center, const Vector& bb_dims,
		double push_dir_angle, const double poi_shift, bool spin) {
	bool f;

	//just consider for now that pushes are done on the x-y plane, specifically on the table
	Vector hand_normal(3);

	//take the hypotenuse of the bounding box cross-section as the worst case scenario
	//similar to a bounding circle actually (radius away from the center)
	double poi_offset = sqrt(bb_dims[0] * bb_dims[0] + bb_dims[1] * bb_dims[1])
			/ 2.0 - 0.01;//0.03 for palm height

	Vector poi_off(3);
	poi_off[0] = 0.0;
	poi_off[1] = 0.0;
	poi_off[2] = 0.0;

	//first convert the angle into simulation coordinate system
	push_dir_angle += PI / 2;

	//make sure that the angle is between [0, 2*PI]
	if (push_dir_angle < 0)
		push_dir_angle += 2 * PI;
	std::cout << "push angle: " << push_dir_angle * 180 / PI << std::endl;

	double des_hand_normal_ang = 0;
	string hand_key;

	if (PI / 4 <= push_dir_angle && push_dir_angle < 3 * PI / 4) {
		des_hand_normal_ang = push_dir_angle + PI;
		hand_key = "karate_hand";
		std::cout << hand_key << std::endl;
	} else if (3 * PI / 4 <= push_dir_angle && push_dir_angle < 5 * PI / 4) {
		des_hand_normal_ang = push_dir_angle + PI / 2;
		//      hand_key = "close_hand";
		hand_key = "fist_hand";
		std::cout << hand_key << std::endl;
	} else if (5 * PI / 4 <= push_dir_angle && push_dir_angle < 7 * PI / 4) {
		des_hand_normal_ang = push_dir_angle;
		hand_key = "karate_hand";
		std::cout << hand_key << std::endl;
	} else {
		des_hand_normal_ang = push_dir_angle - PI / 2.0;
		poi_off[1] = sin(push_dir_angle - PI / 2.0) * poi_offset + sin(
				push_dir_angle + PI) * (-0.035);//0.035 is the half of the palm size
		poi_off[0] = cos(push_dir_angle - PI / 2.0) * poi_offset + cos(
				push_dir_angle + PI) * (-0.035);
		hand_key = "perpendicular_hand";
		std::cout << hand_key << std::endl;
	}

	hand_normal[0] = cos(des_hand_normal_ang);
	hand_normal[1] = sin(des_hand_normal_ang);
	hand_normal[2] = 0;

	poi_off[0] += cos(push_dir_angle + PI) * poi_offset;// x component of the direction vector * reach offset
	poi_off[1] += sin(push_dir_angle + PI) * poi_offset;// y component of the direction vector * reach offset

	//Find the vector-angle representation of this normal
	Vector vec_angle = normal2VectorAngle(hand_normal);

	//    std::cout << bb_center[0] << " " << bb_center[1] << " " << bb_center[2] << std::endl;
	//    std::cout << poi_off[0] << " " << poi_off[1] << " " << poi_off[2] << std::endl;
	//    action->touch (bb_center, vec_angle, poi_off);

	//first go above the point
	poi_off[2] = bb_dims[2] / 2 + 0.02;//0.02cm is object clearance factor
	action->pushAction(bb_center + poi_off, vec_angle, hand_key);
	action->checkActionsDone(f, true);

	//now, get down to the table
	poi_off[2] = -bb_dims[2] + 0.02;//0.02 table clearance factor
	action->pushAction(bb_center + poi_off, vec_angle, hand_key);
	action->checkActionsDone(f, true);

	//it is time to push the object
	Vector poi_sh(3);
	poi_sh[0] = cos(push_dir_angle) * poi_shift;// x component of the direction vector * reach offset
	poi_sh[1] = sin(push_dir_angle) * poi_shift;// y component of the direction vector * reach offset
	if (spin)
		poi_sh[2] = bb_dims[2];//enables spinning object
	else
		poi_sh[2] = 0.0;

	action->pushAction(bb_center + poi_off + poi_sh, vec_angle, hand_key);
	action->checkActionsDone(f, true);

	//get arm directly up if spin is not activated, this enables robot to avoid colliding
	//to the object while doing a homing action etc.
	if (!spin) {
		poi_off[0] += cos(push_dir_angle + PI) * 0.02;
		poi_off[1] += sin(push_dir_angle + PI) * 0.02;
		poi_off[2] += bb_dims[2];
	}

	action->pushAction(bb_center + poi_off + poi_sh, vec_angle, hand_key);
	action->checkActionsDone(f, true);
}

void BehaviorModule::push2(const Vector& bb_center, const Vector& bb_dims,
		double push_dir_angle, const double poi_shift, bool is_left_arm,
		bool spin) {	
	double lim = -0.47;
	cout<<"Will enter according to limit"<<endl;
	cout<<"REACH_X_LIMIT"<<lim<<endl;
	cout<<"bb_upward(0)"<<bb_center[0]<<endl;
	if (bb_center[0] < lim)
			return;
	cout<<"Will move"<<endl;
	//cin.get();
	bool f;

	int coef = is_left_arm ? 1 : -1;
	//just consider for now that pushes are done on the x-y plane, specifically on the table
	Vector hand_normal(3);

	//take the hypotenuse of the bounding box cross-section as the worst case scenario
	//similar to a bounding circle actually (radius away from the center)
	//	double poi_offset = sqrt(bb_dims[0] * bb_dims[0] + bb_dims[1] * bb_dims[1])
	//			/ 2.0 - 0.01;//0.03 for palm height

	double poi_offset = sqrt(bb_dims[0] * bb_dims[0] + bb_dims[1] * bb_dims[1]);

	Vector poi_off(3);
	poi_off[0] = 0.0;
	poi_off[1] = 0.0;
	poi_off[2] = 0.0;

	//first convert the angle into simulation coordinate system
	push_dir_angle += PI / 2;

	//make sure that the angle is between [0, 2*PI]
	if (push_dir_angle < 0)
		push_dir_angle += 2 * PI;
	std::cout << "push angle: " << push_dir_angle * 180 / PI << std::endl;

	double des_hand_normal_ang = 0;
	string hand_key;

	if (PI / 4 <= push_dir_angle && push_dir_angle < 3 * PI / 4) {
		des_hand_normal_ang = push_dir_angle + coef * PI;
		hand_key = "fist_hand";
		std::cout << hand_key << std::endl;
	} else if (3 * PI / 4 <= push_dir_angle && push_dir_angle < 5 * PI / 4) {
		des_hand_normal_ang = push_dir_angle - coef * PI / 2;
		//      hand_key = "close_hand";
		hand_key = "fist_hand";
		std::cout << hand_key << std::endl;
	} else if (5 * PI / 4 <= push_dir_angle && push_dir_angle < 7 * PI / 4) {
		des_hand_normal_ang = push_dir_angle;
		hand_key = "karate_hand";
		std::cout << hand_key << std::endl;
	} else {
		des_hand_normal_ang = push_dir_angle - coef * PI / 2.0;
		poi_off[1] = sin(push_dir_angle - PI / 2.0) * poi_offset + sin(
				push_dir_angle + PI) * (-0.035);//0.035 is the half of the palm size
		poi_off[0] = cos(push_dir_angle - PI / 2.0) * poi_offset + cos(
				push_dir_angle + PI) * (-0.035);
		hand_key = "perpendicular_hand";
		std::cout << hand_key << std::endl;
	}

	hand_normal[0] = cos(des_hand_normal_ang);
	hand_normal[1] = sin(des_hand_normal_ang);
	hand_normal[2] = 0;

	poi_off[0] += cos(push_dir_angle + PI) * poi_offset;// x component of the direction vector * reach offset
	poi_off[1] += sin(push_dir_angle + PI) * poi_offset;// y component of the direction vector * reach offset

	//Find the vector-angle representation of this normal
	//TODO:
	Vector vec_angle = normal2VectorAngle(hand_normal);

	//    std::cout << bb_center[0] << " " << bb_center[1] << " " << bb_center[2] << std::endl;
	//    std::cout << poi_off[0] << " " << poi_off[1] << " " << poi_off[2] << std::endl;
	//    action->touch (bb_center, vec_angle, poi_off);

	//first go above the point
	poi_off[2] = bb_dims[2] / 2.0 + 0.15;//0.15cm is object clearance factor
	std::cout << "***********************************" << std::endl;
	std::cout << "***********************************" << std::endl;
	std::cout << "***********************************" << std::endl;
	std::cout << (bb_center + poi_off)[0] << " " << (bb_center + poi_off)[1]
			<< " " << (bb_center + poi_off)[2] << " " << std::endl;

	int i = 0;
	std::cout << "&&&&&& do you verify the position &&&&&&" << std::endl;
	//cin >> i;

	//	Vector hand_pos;
	//	Vector hand_orient;
	//	action->getPose(hand_pos, hand_orient);
	//check if the object center is between the hand and the first checkpoint
	//this is for push left-right actions
	//proper way to check is to check if this line segment crosses the boundingbox of the object
	//or better way is using 3d collision checking routine here.
	//	if(fabs( (bb_center + poi_off)[1]-hand_pos[1]) > fabs(bb_center[1]-hand_pos[1]))
	//	{
	//		action->pushAction(bb_center + poi_off, vec_angle, hand_key);
	//		action->checkActionsDone(f, true);
	//	}

	action->pushAction(bb_center + poi_off, vec_angle, hand_key);
	action->checkActionsDone(f, true);

	//now, get down to the table
	poi_off[2] = -bb_dims[2] / 2 + 0.08;//0.08 table clearance factor
	std::cout << (bb_center + poi_off)[0] << " " << (bb_center + poi_off)[1]
			<< " " << (bb_center + poi_off)[2] << " " << std::endl;

	std::cout << "&&&&&& do you verify the position &&&&&&" << std::endl;
	//cin >> i;

	action->pushAction(bb_center + poi_off, vec_angle, hand_key);
	action->checkActionsDone(f, true);

	//it is time to push the object
	Vector poi_sh(3);
	poi_sh[0] = cos(push_dir_angle) * poi_shift;// x component of the direction vector * reach offset
	poi_sh[1] = sin(push_dir_angle) * poi_shift;// y component of the direction vector * reach offset
	if (spin)
		poi_sh[2] = bb_dims[2];//enables spinning object
	else
		poi_sh[2] = 0.0;

	std::cout << (bb_center + poi_off + poi_sh)[0] << " " << (bb_center
			+ poi_off + poi_sh)[1] << " " << (bb_center + poi_off + poi_sh)[2]
			<< " " << std::endl;

	std::cout << "&&&&&& do you verify the position &&&&&&" << std::endl;
	//cin >> i;

	action->pushAction(bb_center + poi_off + poi_sh, vec_angle, hand_key);
	action->checkActionsDone(f, true);

	//get arm directly up if spin is not activated, this enables robot to avoid colliding
	//to the object while doing a homing action etc.
	if (!spin) {
		poi_off[0] += cos(push_dir_angle + PI) * 0.02;
		poi_off[1] += sin(push_dir_angle + PI) * 0.02;
		poi_off[2] += bb_dims[2];
	}

	std::cout << (bb_center + poi_off + poi_sh)[0] << " " << (bb_center
			+ poi_off + poi_sh)[1] << " " << (bb_center + poi_off + poi_sh)[2]
			<< " " << std::endl;

	std::cout << "&&&&&& do you verify the position &&&&&&" << std::endl;
	//cin >> i;

	action->pushAction(bb_center + poi_off + poi_sh, vec_angle, hand_key);
	action->checkActionsDone(f, true);
}

void BehaviorModule::lift(const double poi_shift) {
	Vector hand_pos;
	Vector hand_orient;
	action->getPose(hand_pos, hand_orient);
	hand_pos[2] += poi_shift;
	action->disableContactDetection();
	action->pushAction(hand_pos, hand_orient);
}

void BehaviorModule::hide(Vector bb_center, Vector bb_dims) {
	Vector poi_off(3);
	poi_off[0] = 0.0;
	poi_off[1] = 0.0;
	poi_off[2] = bb_dims[2] / 2 + 0.02;//half of the height + safety distance

	Vector orient = angleXZToVectorAngle(0, PI);
	action->pushAction(bb_center + poi_off, orient, "karate_hand");
}

void BehaviorModule::pointTo(Vector point) {
	Vector left_shoulder(3);
	left_shoulder[0] = 0.0;
	left_shoulder[1] = -0.12;//to the left (crouch-left shoulder)
	left_shoulder[2] = 0.25;// to the up (crouch-neck)

	Vector shoulder_to_point = point - left_shoulder;
	double delta_dist = sqrt(pow(shoulder_to_point[0], 2) + pow(
			shoulder_to_point[1], 2) + pow(shoulder_to_point[2], 2));

	//if the point is not reachable
	if (delta_dist > 0.35) {
		Vector des_position = (point - left_shoulder) * 0.35 / delta_dist
				+ left_shoulder;
		Vector normalized_dir = (point - left_shoulder) / delta_dist;
		std::cout << normalized_dir[0] << " " << normalized_dir[1] << " "
				<< normalized_dir[2] << std::endl;

		double z_angle = atan2(normalized_dir[1], normalized_dir[0]);
		if (z_angle < 0)
			z_angle += 2 * PI;
		std::cout << z_angle / PI * 180 << std::endl;
		double y_angle = atan2(normalized_dir[2], sqrt(
				pow(normalized_dir[0], 2) + pow(normalized_dir[1], 2)));
		std::cout << y_angle / PI * 180 << std::endl;

		//      Vector des_orient = angleXZToVectorAngle(-PI/2, z_angle);

		Vector des_orient = angleXYZToVectorAngle(-PI / 2, y_angle, z_angle);
		//      Vector des_orient = angleXYZToVectorAngle (0 , -PI/6,  PI);

		//action->disableTorsoDof ();
		action->pushAction(des_position, des_orient, "point_hand", 5);
		//action->enableTorsoDof ();
	}
}

	void
	BehaviorModule::releaseUpward(Vector point)
	{
		release (point, false);
	}
	
	void
	BehaviorModule::releaseDownward(Vector point)
	{
		release (point, true);
	}

void BehaviorModule::release(Vector point, bool palm_upward) {
	Vector hand_orient;

//	if (palm_upward)
//		hand_orient = angleXZToVectorAngle(PI, PI);
//	else
//		hand_orient = angleXZToVectorAngle(0, PI);

	if (palm_upward)
		hand_orient = angleXZToVectorAngle(PI, PI);
	else
		hand_orient = angleXZToVectorAngle(0, PI);

	bool f;
	std::cout<<"going to the location: "<<point[0]<<" "<<point[1]<<" "<<point[2]<<std::endl;
	//cin.get();
	action->pushAction(point, hand_orient);
	action->checkActionsDone(f, true);
	// release the object or just open the
	// hand (wait until it's done)
	action->pushAction("open_hand");
	action->areFingersInPosition(f);
	action->checkActionsDone(f, true);
}

void BehaviorModule::doReflexiveGrasp() {

}

void BehaviorModule::grasp(Vector bb_center) {
	Vector xv(3);
	xv[0] = 0.0;
	xv[1] = 0.0;
	xv[2] = -0.0;

	//check if fingers contact
	//    bb_center[0]+= 0.02;
	bb_center[2] -= 0.03;

	bool f;

	// go and grasp (wait until it's done)
	action->enableContactDetection();
	action->pushAction("open_hand");
	action->grasp(bb_center, graspOrien, graspDisp, xv);
	action->checkActionsDone(f, true);
	action->areFingersInPosition(f);
}

void BehaviorModule::graspLiftAndRelease(Vector bb_center,
		Vector bb_target_center, const double shift, bool palm_upward) {
	grasp(bb_center);
	lift(shift);
	release(bb_target_center, palm_upward);
}

void BehaviorModule::home(bool is_left_arm) {
	bool f, dn;
	Vector home_orient = angleXZToVectorAngle(-2 * PI / 5, PI);
	Vector js, home_coords;
	js.resize(positions_torso_enc.size());

	js[0] = 0;
	js[1] = 0;
	js[2] = 0;

	//for (int i = 0; i < js.size(); i++)
	//	js[i] = positions_torso_enc[i];

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

	action->pushAction(home_coords, home_orient, "open_hand");
	action->checkActionsDone(f, true);

}

void BehaviorModule::tuckArms() {

	//	home();

	action->pushAction("open_hand");
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

	//first disconnect cartesian solvers
	//	if (!sim) {
	if (left_arm_cart_solver_active) {
		Network::disconnect(
				"/actionPrimitivesMod/left_arm/position/command:o",
				("/" + robotName + "cartesianController/right_arm/command:i").c_str());
		Network::disconnect("/actionPrimitivesMod/left_arm/position/rpc:o",
				("/" + robotName + "left_arm/rpc:i").c_str());
		Network::disconnect(("/" + robotName + "left_arm/state:o").c_str(),
				"/actionPrimitivesMod/left_arm/position/state:i");
		//      driver_left.open (options_left);

	}

	if (right_arm_cart_solver_active) {
		Network::disconnect(
				"/actionPrimitivesMod/right_arm/position/command:o",
				("/" + robotName + "cartesianController/right_arm/command:i").c_str());
		Network::disconnect("/actionPrimitivesMod/right_arm/position/rpc:o",
				("/" + robotName + "right_arm/rpc:i").c_str());
		Network::disconnect(("/" + robotName + "right_arm/state:o").c_str(),
				"/actionPrimitivesMod/right_arm/position/state:i");
		//      driver_right.open (options_right);
	}
	//	} else {
	//		//TODO for simulator
	//
	//	}
	//	std::cout << " ************** " << std::endl;

	//first home the torso
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

	//now send target joint angles, assuming that it is already safe to do so
	//	encoders_left->getEncoders(positions_left_enc.data());
	//	std::cout << " --------------- " << std::endl;
	encoders_right->getEncoders(positions_right_enc.data());
	//	std::cout << " !!!!!!!!!!!!!!! " << std::endl;

	//assuming that both arms having the same #DOF
	for (int i = 0; i < positions_left_enc.size(); i++) {
		//		pos_ctrl_left->setRefSpeed(i, 10.0);
		pos_ctrl_right->setRefSpeed(i, 10.0);

		//		pos_ctrl_left->setRefAcceleration(i, 50.0);
		pos_ctrl_right->setRefAcceleration(i, 50.0);
	}
	//set command positions
	//	Vector js;
	js.resize(positions_right_enc.size());
	std::cout << " ************** " << js.size() << " ************** "
			<< std::endl;
	//js[0] = -15;
	js[0] = 10;
	js[1] = 20;
	//js[1] = 40;//for simulator
	js[2] = -30;
	js[3] = 60;
	js[4] = -60;
	js[5] = 0;
	js[6] = 20;
	//	for (int i = 7; i < js.size(); i++)
	//		js[i] = positions_left_enc[i];
	//	pos_ctrl_left->positionMove(js.data());

	for (int i = 7; i < js.size(); i++)
		js[i] = positions_right_enc[i];
	pos_ctrl_right->positionMove(js.data());

	bool done = false;
	//	while (!done && left_arm_cart_solver_active) {
	//		pos_ctrl_left->checkMotionDone(&done);
	//		Time::delay(0.001);
	//	}

	done = false;
	while (!done && right_arm_cart_solver_active) {
		pos_ctrl_right->checkMotionDone(&done);
		Time::delay(0.001);
	}

	js[0] = 10;
	pos_ctrl_right->positionMove(js.data());
	done = false;
	while (!done && right_arm_cart_solver_active) {
		pos_ctrl_right->checkMotionDone(&done);
		Time::delay(0.001);
	}

	//finally connect cartesian solvers back
	//if (!sim) {
	if (left_arm_cart_solver_active) {

		//      driver_left.close();

		Network::connect(
				"/actionPrimitivesMod/left_arm/position/command:o",
				("/" + robotName + "cartesianController/right_arm/command:i").c_str());
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
	//	} else {
	//		//TODO for simulator
	//	}
}

void BehaviorModule::lookAtPoint(Vector bb_center) {

	int i;
	std::cout << bb_center[0] << " " << bb_center[1] << " " << bb_center[2]
			<< std::endl;
	std::cout << "&&&&&& do you verify the position &&&&&&" << std::endl;
	//cin >> i;

	igaze->lookAtFixationPoint(bb_center);
	bool done = false;
	while (!done) {
		//igaze->checkMotionDone(&done);
		//Time::delay(0.04); // or any suitable delay
		//sleep(1);
		done = igaze->waitMotionDone(0.2, 0.0);
	}
	sleep(3);
	std::cout << "lookAtPoint finished!" << std::endl;
}

void BehaviorModule::lookAtRegion(uint region_id) {
	//TODO: Yigidim aslanim
	if (region_id == 1) {

	} else if (region_id == 2) {

	}
	//	pos_ctrl_head->positionMove
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
		//igaze->checkMotionDone(&done);
		//Time::delay(0.04); // or any suitable delay
		//sleep(1);
		done = igaze->waitMotionDone(0.2, 0.0);
	}
	sleep(3);
	std::cout << "lookAtPoint finished!" << std::endl;
}

int BehaviorModule::voiceCommand(Vector bb_center, Vector bb_dims) {
	Bottle* b = port_simon_in.read(true);
	int cmd = b->get(0).asInt();

	//    if(new_cmd_data_rcvd)
	//    {
	//      new_cmd_data_rcvd= false;
	//      cmd = cmd_id;
	//    }

	bool f;

	if (cmd == 1)//icub do push_left
	{
		push(bb_center, bb_dims, PI, 0.10);
	} else if (cmd == 2)//icub do push_right
	{
		push(bb_center, bb_dims, 0, 0.10);
	} else if (cmd == 3)//icub do rest
	{
		action->disableContactDetection();
		action->pushAction(home_x, "open_hand");
		action->checkActionsDone(f, true);
		//		action->enableContactDetection();
	} else if (cmd == 4)//icub do push_forward
	{
		push(bb_center, bb_dims, PI / 2, 0.10);
	} else if (cmd == 5)//icub do push_backward
	{
		push(bb_center, bb_dims, 3 * PI / 2, 0.10);
	} else if (cmd == 6)//icub do point
	{
		bb_center[0] = -0.5;
		bb_center[1] = -0.2;
		bb_center[2] = 0.1;
		pointTo(bb_center);
	} else if (cmd == 7)//icub do hide
	{
		hide(bb_center, bb_dims);
	}
	/*
	 else if(s == "icub do pushforward")
	 cmd = 4;
	 else if(s == "icub do pushbackward")
	 cmd = 5;
	 else if(s == "icub do point")
	 cmd = 6;
	 else if(s == "icub do hide")
	 cmd = 7;
	 */
	return cmd;
}

void BehaviorModule::manualCommand(Vector bb_center, Vector bb_dims) {
	int op_type;
	bool invalid_op = false;
	int push_direction;

	//    createObject (0, bb_center);
	//    pointTo (bb_center);

	bool f;

	while (true) {

		std::cout << "Select the behavior number: " << std::endl;
		std::cout << "Behavior #1: Point to the specified object" << std::endl;
		std::cout << "Behavior #2: Grasp the specified object" << std::endl;
		std::cout << "Behavior #3: Lift the specified object" << std::endl;
		std::cout << "Behavior #4: Release the specified object" << std::endl;
		std::cout << "Behavior #5: Hide the specified object" << std::endl;
		std::cout << "Behavior #6: Push the specified object" << std::endl;
		std::cout
				<< "Behavior #7: Grasp, lift and release the specified object"
				<< std::endl;
		std::cout << "Behavior #8: Shake the specified object" << std::endl;
		std::cout << "Behavior #9: Basket the specified object" << std::endl;
		std::cout << "Behavior #10: Go to home" << std::endl;
		std::cout << "Behavior #11: push through tunnel" << std::endl;
		if (invalid_op)
			std::cout
					<< "INVALID BEHAVIOR NUMBER. Please re-enter the behavior number: ";
		else
			std::cout << "Enter number: ";
		std::cin >> op_type;

		deleteObject();
		Vector v;
		switch (op_type) {
		case 1:
			createObject(0, bb_center);
			pointTo(bb_center);
			break;
		case 2:
			createObject(1, bb_center);
			grasp(bb_center);
			break;
		case 3:
			lift(0.25);
			break;
		case 4:
			release(bb_center);
			deleteObject();
			break;
		case 5:
			createObject(1, bb_center);
			hide(bb_center, bb_dims);
			deleteObject();
			break;
		case 6:
			createObject(1, bb_center);
			std::cout
					<< "Enter which direction you want to push the object (forward=1, backward=2, left=3, right=4): ";
			std::cin >> push_direction;
			if (push_direction == 1) {
				push(bb_center, bb_dims, PI / 2, 0.10);
				action->disableContactDetection();
				action->pushAction(home_x, "open_hand");
				action->checkActionsDone(f, true);
				//				action->enableContactDetection();
			} else if (push_direction == 2) {
				push(bb_center, bb_dims, 3 * PI / 2, 0.10);
				action->disableContactDetection();
				action->pushAction(home_x, "open_hand");
				action->checkActionsDone(f, true);
				//				action->enableContactDetection();
			} else if (push_direction == 3) {
				push(bb_center, bb_dims, PI, 0.10);
				action->disableContactDetection();
				action->pushAction(home_x, "open_hand");
				action->checkActionsDone(f, true);
				//				action->enableContactDetection();
			} else if (push_direction == 4) {
				push(bb_center, bb_dims, 0, 0.10);
				action->disableContactDetection();
				action->pushAction(home_x, "open_hand");
				action->checkActionsDone(f, true);
				//				action->enableContactDetection();
			}
			deleteObject();
			break;
		case 7:
			createObject(1, bb_center);
			v = bb_center;
			v[2] += 0.10;
			graspLiftAndRelease(bb_center, v, 0.10, true);
			break;
		case 8:
			break;
		case 9:
			createObject(1, bb_center);
			basket(bb_center, bb_dims);
			break;
		case 10:
			home();
			break;
		case 11:
			createObject(2, bb_center);
			action->disableContactDetection();
			push(bb_center, bb_dims, 0, 0.10, true);
			action->pushAction(home_x, "open_hand");
			action->checkActionsDone(f, true);
			//			action->enableContactDetection();
			break;
		default:
			invalid_op = true;
		}
		if (!invalid_op)
			break;
	}

	bool done;
	action->checkActionsDone(done, true);
	std::cout << "ACTIONS DONE" << std::endl;
}

//from top or sides (left, right, back)
void BehaviorModule::fetch(Vector bb_center, Vector bb_dims, Vector hand_orient) {
	grasp(bb_center);
	home();
}

void BehaviorModule::shake(const Vector dir, const double shake_off) {
	Vector delta_point(3);
	delta_point[0] = dir[0] * shake_off;
	delta_point[1] = dir[1] * shake_off;
	delta_point[2] = dir[2] * shake_off;

	Vector hand_pos;
	Vector hand_orient;
	action->getPose(hand_pos, hand_orient);

	for (uint i = 0; i < 10; i++) {
		action->pushAction(hand_pos + delta_point, 3);
		action->pushAction(hand_pos - delta_point, 3);
	}
}

void BehaviorModule::basket(Vector bb_center, Vector bb_dims) {
	//basket position: -0.45 0 0.0016

	Vector basket_pos(3);
	double lift_off = 0.15;
	basket_pos[0] = -0.45 - 0.03;//0.03 for palm
	basket_pos[1] = 0.0;
	basket_pos[2] = 0.0016 + lift_off;
	graspLiftAndRelease(bb_center, basket_pos, lift_off, false);
}

void BehaviorModule::testHandSequences() {
	bool f;

	action->pushAction("close_hand");
	action->checkActionsDone(f, true);
	action->areFingersInPosition(f);

	action->pushAction("open_hand");
	action->checkActionsDone(f, true);
	action->areFingersInPosition(f);

	action->pushAction("close_hand");
	action->checkActionsDone(f, true);
	action->areFingersInPosition(f);

	action->pushAction("karate_hand");
	action->checkActionsDone(f, true);
	action->areFingersInPosition(f);
}

void BehaviorModule::createObject(int op_type, Vector& bb_center) {
	Bottle& obj = port_sim_rpc_out.prepare();
	obj.clear();
	string object_type;
	double a, b, c;
	double r;
	double red, green, blue;
	double x, y, z;
	int side;

	y = 0.53;
	r = 0.03;
	b = c = 0.06;
	a = 0.04;
	red = 1;
	green = blue = 0;

	std::cout << "Select the object type (sph, box): ";
	std::cin >> object_type;

	if (op_type == 0) { // point to behaviour
		std::cout << "Which side of the table (far=0, near=1): ";
		std::cin >> side;

		if (side == 0) {
			z = 0.6;
		} else {
			z = 0.3;
		}
		x = 0.3;
	} else if (op_type == 1) {
		std::cout << "Which side of the table (left=0, middle=1, right=2): ";
		std::cin >> side;
		if (side == 0) {
			x = 0.2;
			//        x = 0.5;
		} else if (side == 1) {
			x = 0;
		} else {
			x = -0.1;
		}
		z = 0.3;
		//      z = 0.6;
	} else {
		x = 0.10;
		z = 0.35;
	}

	obj.addString("world");
	obj.addString("mk");
	obj.addString(object_type.c_str());

	if (object_type == "sph") {
		obj.addDouble(r);
	} else {
		obj.addDouble(a);
		obj.addDouble(b);
		obj.addDouble(c);
	}

	obj.addDouble(x);
	obj.addDouble(y);
	obj.addDouble(z);

	bb_center[0] = -z - 0.04;
	bb_center[1] = -x - 0.02;
	bb_center[2] = y - 0.5484;

	obj.addDouble(red);
	obj.addDouble(green);
	obj.addDouble(blue);

	port_sim_rpc_out.write();
}

void BehaviorModule::deleteObject() {
	Bottle& obj = port_sim_rpc_out.prepare();
	obj.clear();

	obj.addString("world");
	obj.addString("del");
	obj.addString("all");

	port_sim_rpc_out.write();
}

// we don't need a thread since the actions library already
// incapsulates one inside dealing with all the tight time constraints
bool BehaviorModule::updateModule() {

	// do it only once
	if (firstRun) {
		init();
		firstRun = false;
	}
	//	std::cout << "executing update module..." << std::endl;

	Vector bb_dims(3);
	bb_dims[0] = 0.06;
	bb_dims[1] = 0.06;
	bb_dims[2] = 0.06;

	// get a target object position from a YARP port
	// Bottle *b = inPort.read (); // blocking call

	Vector bb_center(3);
	//	bb_center[0] = -0.3;
	//	bb_center[1] = -0.2;
	//	bb_center[2] = 0.1;

	bb_center[0] = -0.30;
	bb_center[1] = 0.15;
	bb_center[2] = 0.20;

	//	action->pushAction(bb_center, "open_hand");
	//	action->checkActionsDone(f, true);

	//	tuckArms();

	//	push2(bb_center, bb_dims, 0, 0.10, false, false);

	//	menu_mutex.wait();
	//    voiceCommand (bb_center, bb_dims);
	//	manualCommand(bb_center, bb_dims);

	//	hide(bb_center, bb_dims);
	//	menu_mutex.post();

	ros::spinOnce();
	return true;
}

bool BehaviorModule::interruptModule() {
	// since a call to checkActionsDone() blocks
	// the execution until it's done, we need to
	// take control and exit from the waiting state
	action->syncCheckInterrupt(true);

	inPort.interrupt();
	rpcPort.interrupt();

	return true;
}
