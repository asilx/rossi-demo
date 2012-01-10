// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2010 RobotCub Consortium
 * Author: Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/all.h>
#include <iostream>

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[]) {
	Network yarp;
	BufferedPort<Bottle> port;
	port.open("/port_writer");
	while (true) {
		cout << "waiting for input" << endl;
		Bottle *input = port.read();

		if (input != NULL) {
			cout << "got " << input->toString().c_str() << endl;

	    	Bottle& output = port.prepare();
    		output.clear();
    		
			for (int i = 0; i < input->size(); i++)
				output.addDouble(input->get(i).asDouble());

			cout << "wrote " << output.toString().c_str() << endl;
			port.write();
		}
	}
	return 0;
}
