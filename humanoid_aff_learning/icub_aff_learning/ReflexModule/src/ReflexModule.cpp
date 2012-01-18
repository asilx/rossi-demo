
#include "ReflexModule.h"

bool ReflexModule::updateModule()
{
	yarp::os::Bottle* points = _portPC.read();
	double totalReading = 0;
	for (int i = 8; i < 12; i++) 
	{
		for (int j=0;j<12;j++)
		{
			totalReading += points->get(i*12+j).asDouble();
			
		}
	}
	
	if(totalReading > 30 && !isClosed)
	{
		isClosed = true;
		yarp::os::Bottle& btout = _portFeat.prepare();
		btout.clear();
		
		std::cout << "Closing the hand..." << std::endl;
		
		std::string s = "gs";
		btout.addString(s.c_str());
		_portFeat.write();
	
	}
	else if(totalReading <= 30 && isClosed)
	{
		isClosed = false;
		
		yarp::os::Bottle& btout = _portFeat.prepare();
		btout.clear();
		
		std::cout << "Opening the hand..." << std::endl;
		
		std::string s = "oh";
		btout.addString(s.c_str());
		_portFeat.write();
	
	}

	
	return true;
}

ReflexModule::ReflexModule()
{
	
}

double 	ReflexModule::getPeriod()
{
	return 0.5;
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
	isClosed = false;
	return true;
}
