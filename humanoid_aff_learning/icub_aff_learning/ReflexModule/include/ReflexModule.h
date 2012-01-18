#ifndef REFLEXMODULE_H_
#define REFLEXMODULE_H_


#include "string"
#include <signal.h> 

#include "cstdio"
#include <iostream>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <sstream>

#include "sys/stat.h"

using namespace std;

class ReflexModule : public yarp::os::RFModule
{
public:


    virtual bool 	configure(yarp::os::ResourceFinder &rf);
    virtual bool 	close();
    virtual bool 	interruptModule();
    virtual bool 	updateModule();
    virtual bool 	respond(const yarp::os::Bottle &command,yarp::os::Bottle &reply);
    virtual double 	getPeriod();

	ReflexModule();


private:
	yarp::os::BufferedPort<yarp::os::Bottle> _portFeat;
	yarp::os::BufferedPort<yarp::os::Bottle> _portPC;
        bool isClosed;
	
};

#endif 
