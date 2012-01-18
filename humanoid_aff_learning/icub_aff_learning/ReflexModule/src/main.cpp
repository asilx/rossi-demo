#include "ReflexModule.h"

#define DATA_PRECISION ROUGH

using namespace std;


//enter ip of the camera as the second command line argument
int main(int argc, char** argv)
{
	int isRight = atoi(argv[1]);
	yarp::os::Network yarp;
	yarp::os::ResourceFinder rf;
	rf.setVerbose(true);

	ReflexModule sr4k;

    return sr4k.runModule(rf);
}
