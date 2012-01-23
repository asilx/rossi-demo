#include <yarp/os/Time.h>
#include "ReflexModule.h"

bool ReflexModule::updateModule()
{
  	// using tactile info for reflexive action
	// yarp::os::Bottle* points = _portPC.read();
	// double totalReading = 0;
	// for (int i = 0; i < 12; i++)
	// {
	// 	for (int j=0;j<12;j++)
	// 	{
	// 		totalReading += points->get(i*12+j).asDouble();

	// 	}
	// }
	// cout << totalReading << endl;

	// if(totalReading > 20 && !isClosed)
	// {
	// 	isClosed = true;
	// 	yarp::os::Bottle& btout = _portFeat.prepare();
	// 	btout.clear();

	// 	std::cout << "Closing the hand..." << std::endl;

	// 	std::string s = "gs";
	// 	btout.addString(s.c_str());
	// 	_portFeat.write();
	// 	yarp::os::Time::delay(3.0);

	// }
	// else if(totalReading <= 20 && isClosed)
	// {
	// 	isClosed = false;

	// 	yarp::os::Bottle& btout = _portFeat.prepare();
	// 	btout.clear();

	// 	std::cout << "Opening the hand..." << std::endl;

	// 	std::string s = "oh";
	// 	btout.addString(s.c_str());
	// 	_portFeat.write();
	// 	yarp::os::Time::delay(3.0);

	// }


	// using force torque for reflexive action

	yarp::os::Bottle* forceBottle = forcePort.read();
	double zAxisForce = forceBottle->get(2).asDouble();
	std::cout << "Z axis force is: " << zAxisForce << endl;
	if (zAxisForce > 2.0)
	  {
	    isClosed = true;
	    yarp::os::Bottle& btout = _portFeat.prepare();
	    btout.clear();

	    std::cout << "Closing the hand..." << std::endl;

	    std::string s = "gs";
	    btout.addString(s.c_str());
	    _portFeat.write();
	    yarp::os::Time::delay(3.0);

	  }
	else
	  {
	    isClosed = false;
	    yarp::os::Bottle& btout = _portFeat.prepare();
	    btout.clear();

	    std::cout << "Opening the hand..." << std::endl;

	    std::string s = "oh";
	    btout.addString(s.c_str());
	    _portFeat.write();
	    yarp::os::Time::delay(3.0);

	return true;
}

ReflexModule::ReflexModule(int i)
{
	isRight = i;
}

double 	ReflexModule::getPeriod()
{
	return 1;
}

bool 	ReflexModule::close()
{
	return true;
}

bool 	ReflexModule::interruptModule()
{
	_portFeat.interrupt();
	_portPC.interrupt();
	return true;
}

bool 	ReflexModule::respond(const yarp::os::Bottle &command,yarp::os::Bottle &reply)
{
	return true;
}

bool	ReflexModule::configure(yarp::os::ResourceFinder &rf)
{
	_portFeat.open(getName("/o:ReflexCommand"));
	_portPC.open(getName("/i:ReflexTactile"));
	forcePort.open("/i:reflexiveForce")

	if(isRight)
	{
		yarp::os::Network::connect("/o:ReflexCommand", "/tactGraspRight/rpc:i");
		yarp::os::Network::connect("/icub/skin/righthandcomp", "/i:ReflexTactile");
		yarp::os::Network::connect("/wholeBodyDynamics/right_arm/endEffectorWrench", "/i:ReflexiveForce");
	}
	else
	{
		yarp::os::Network::connect("/o:ReflexCommand", "/tactGraspLeft/rpc:i");
		yarp::os::Network::connect("/icub/skin/lefthandcomp", "/i:ReflexTactile");
		yarp::os::Network::connect("/wholeBodyDynamics/left_arm/endEffectorWrench", "/i:ReflexiveForce");
	}

	isClosed = false;
	return true;
}
