#include <stdio.h>
#include <iostream>
#include <string.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <gsl/gsl_math.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>  //for dcm2axis function
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class CtrlThread: public RateThread
{
protected:
    PolyDriver         client;
    PolyDriver		   clientR;
    ICartesianControl *arm;
    ICartesianControl *right_arm;
    IPositionControl *armp[2];
    
    PolyDriver robot0, robot1;
    
    Property option[2];
    
    IEncoders *encs1;
    IEncoders *encs2;
    
    BufferedPort<Vector> position;

    Vector xd;
    Vector od;
    
    Vector beginXD, beginXDR;
    Vector beginOD, beginODR;

    int startup_context_id;

    double sw, dY;

public:
    CtrlThread(const double period) : RateThread(int(period*1000.0)) { }

    virtual bool threadInit()
    {
		Network yarp;
		position.open("/client/pos");
		Network::connect("/target/pos", "/client/pos");
		
        Property optionL("(device cartesiancontrollerclient)");
        optionL.put("remote","/icubSim/cartesianController/left_arm");
        optionL.put("local","/cartesian_client/left_arm");
        
        Property optionR("(device cartesiancontrollerclient)");
        optionR.put("remote", "/icubSim/cartesianController/right_arm");
        optionR.put("local", "/cartesian_client/right_arm");
        
        option[0].put("device", "remote_controlboard");
		option[0].put("local", "/client/pickUp1");
		option[0].put("remote", "/icubSim/left_arm");
		option[1].put("device", "remote_controlboard");
		option[1].put("local", "/client/pickUp2");
		option[1].put("remote", "/icubSim/right_arm");
		
		robot0.open(option[0]);
		robot1.open(option[1]);
		
		if(!robot0.isValid() || !robot1.isValid()) {
			cout << "Connection cannot be established with arm!" << endl;
			return (-1);
		}
		
		bool ok = robot0.view(armp[0]) && robot0.view(encs1) && robot1.view(armp[1]) && robot1.view(encs2);
		
		if(!ok) {
			cout << "Cannot get interface arm!" << endl;
			robot0.close();
			robot1.close();
			return -1;
		}

		sw = 0.0;
		dY = 0.1;

        if (!client.open(optionL))
            return false;
        if(!clientR.open(optionR))
			return false;

        // open the view
        client.view(arm);
        clientR.view(right_arm);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        arm->storeContext(&startup_context_id);
        right_arm->storeContext(&startup_context_id);

        // set trajectory time
        arm->setTrajTime(1.0);
        right_arm->setTrajTime(1.0);

        // get the torso dofs
        Vector newDof, curDof;
        Vector newDofR, curDofR;
        arm->getDOF(curDof);
        right_arm->getDOF(curDofR);
        newDof=curDof;
        newDofR=curDofR;
        

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=1;
        newDof[1]=0;
        newDof[2]=1;
        newDofR[0]=1;
        newDofR[1]=0;
        newDofR[2]=1;

        // impose some restriction on the torso pitch
        limitTorsoPitch();

        // send the request for dofs reconfiguration
        arm->setDOF(newDof,curDof);
        right_arm->setDOF(newDofR, curDofR);

        xd.resize(3);
        od.resize(4);
        beginXD.resize(3);
        beginOD.resize(4);
        beginODR.resize(4);
        beginXDR.resize(3);
        arm->getPose(beginXD, beginOD);
        right_arm->getPose(beginXDR, beginODR);

        return true;
    }
    
    void waitUntilFinish(ICartesianControl* car) {
		bool done = false;
		while(!done) {
			car->checkMotionDone(&done);
			Time::delay(0.01);
		}
	}
	
	void waitUntilFinish(IPositionControl* pos) {
		bool done = false;
		while(!done) {
			pos->checkMotionDone(&done);
			Time::delay(0.01);
		}
	}
	
	void disconnectCartesian(char ch) {
		if(ch == 'l') {
			Network::disconnect("/icubSim/cartesianController/left_arm/state:o", "/cartesian_client/left_arm/state:i");
			Network::disconnect("/cartesian_client/left_arm/rpc:o", "/icubSim/cartesianController/left_arm/rpc:i");
			Network::disconnect("/cartesian_client/left_arm/command:o", "/icubSim/cartesianController/left_arm/command:i");
		}
		else if(ch == 'r') {
			Network::disconnect("/icubSim/cartesianController/right_arm/state:o", "/cartesian_client/right_arm/state:i");
			Network::disconnect("/cartesian_client/right_arm/rpc:o", "/icubSim/cartesianController/right_arm/rpc:i");
			Network::disconnect("/cartesian_client/right_arm/command:o", "/icubSim/cartesianController/right_arm/command:i");			
		}
	}
	
	void connectCartesian(char ch) {
		if(ch == 'l') {
			Network::connect("/icubSim/cartesianController/left_arm/state:o", "/cartesian_client/left_arm/state:i");
			Network::connect("/cartesian_client/left_arm/rpc:o", "/icubSim/cartesianController/left_arm/rpc:i");
			Network::connect("/cartesian_client/left_arm/command:o", "/icubSim/cartesianController/left_arm/command:i");
		}
		else if(ch == 'r') {
			Network::connect("/icubSim/cartesianController/right_arm/state:o", "/cartesian_client/right_arm/state:i");
			Network::connect("/cartesian_client/right_arm/rpc:o", "/icubSim/cartesianController/right_arm/rpc:i");
			Network::connect("/cartesian_client/right_arm/command:o", "/icubSim/cartesianController/right_arm/command:i");			
		}
	}

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Thread started successfully\n");
        else
            fprintf(stdout,"Thread did not start\n");
    }

    virtual void run()
    {
        generateTarget();
    }

    virtual void threadRelease()
    {    
        // we require an immediate stop
        // before closing the client for safety reason
        arm->stopControl();
		right_arm->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        arm->restoreContext(startup_context_id);
        right_arm->restoreContext(startup_context_id);

        client.close();
        clientR.close();
    }

    void generateTarget()
    {   	
		Vector *pos = position.read();
		sw = 0.0;
		dY = 0.3;
		
		double dZ = 0.0;
		
		if(pos != NULL) {
			if((*pos)[4] == 1) {
				xd[0] = - (*pos)[3] - 0.04;
				xd[1] = - (*pos)[1];
				xd[2] = (*pos)[2] - 0.57 + 0.01 + (*pos)[0];
				cout << xd[0] << "\t" << xd[1] << "\t" << xd[3] << endl;
				od[0] = 0.0;
				od[1] = 0.0;
				od[2] = 1.0;
				od[3] = M_PI;
				if(xd[1] > 0) {
					od[0] = 0.0;
					od[1] = 1.0;
					od[2] = 0.0;
					right_arm->goToPose(xd, od);
					waitUntilFinish(right_arm);
					xd[2] -= 0.02;
					right_arm->goToPose(xd, od);
					waitUntilFinish(right_arm);
				}
				else {
					arm->goToPose(xd,od);
					waitUntilFinish(arm);
					xd[2] -= 0.02;
					arm->goToPose(xd, od);
					waitUntilFinish(arm);
				}
			}
			else if((*pos)[4] == 0){
				while(true) {
					xd[0] = - (*pos)[3] - 0.025;
					xd[1] = - (*pos)[1] + sw + (*pos)[0] + 0.08;
					if(xd[1] < -0.25)
						break;
					xd[2] = (*pos)[2] - 0.585 + dY + dZ;
					
					Matrix m;
					m.resize(3, 3);
					Vector tmp;
					tmp.resize(3);
					tmp[0] = -1;
					tmp[1] = 0;
					tmp[2] = 0;
					m.setCol(0, tmp);
					tmp[0] = 0;
					tmp[1] = -sin(M_PI/4);
					tmp[2] = -cos(M_PI/4);
					m.setCol(1, tmp);
					tmp[1] = -cos(M_PI/4);
					tmp[2] = sin(M_PI/4);
					m.setCol(2, tmp);
					od = iCub::ctrl::dcm2axis(m);
					if(xd[1] > 0.5) {
						//disconnectCartesian('l');
						//for(int i = 0; i < 16; i++)
							//armp[0]->positionMove(i, 0);
						//waitUntilFinish(armp[0]);
						//connectCartesian('l');
						
						xd[1] = - (*pos)[1] - sw - (*pos)[0] - 0.09;
						if(xd[1] > 0.25) break;
						cout << xd[0] << "\t" << xd[1] << "\t" << xd[3] << endl;
						if(dY == 0.3)
							sw -= 0.02;
						dZ += 0.005;
						right_arm->goToPose(xd, od);
						waitUntilFinish(right_arm);
						if(dY == 0)
							sw -= 0.08;
					}
					else {
						//disconnectCartesian('r');
						//for(int i = 0; i < 16; i++)
							//armp[1]->positionMove(i, 0);
						//waitUntilFinish(armp[1]);
						//connectCartesian('r');
						cout << xd[0] << "\t" << xd[1] << "\t" << xd[3] << endl;
						if(dY == 0.3)
							sw -= 0.02;
						dZ += 0.005;
						arm->goToPose(xd, od);
						waitUntilFinish(arm);
						if(dY == 0)
							sw -= 0.08;
					}
					dY = 0.0;
				}
				arm->goToPose(beginXD, beginOD);
				waitUntilFinish(arm);
				right_arm->goToPose(beginXDR, beginODR);
				waitUntilFinish(right_arm);
				waitUntilFinish(right_arm);
			}
			else if((*pos)[4] == 2) {
				arm->goToPose(beginXD, beginOD);
				waitUntilFinish(arm);
				right_arm->goToPose(beginXDR, beginODR);
				waitUntilFinish(right_arm);
			}
			else if((*pos)[4] == 3) {
				xd[0] = - (*pos)[3] - 0.04;
				xd[1] = - (*pos)[1];
				xd[2] = (*pos)[2] - 0.56 + (*pos)[0];
				cout << xd[0] << "\t" << xd[1] << "\t" << xd[3] << endl;
				od[0] = 0.0;
				od[1] = 0.0;
				od[2] = 1.0;
				od[3] = M_PI;
				if(xd[1] > 0) {
					od[0] = 0.0;
					od[1] = 1.0;
					od[2] = 0.0;
					right_arm->goToPose(xd, od);
					waitUntilFinish(right_arm);
					xd[0] += 0.04;
					right_arm->goToPose(xd, od);
					waitUntilFinish(right_arm);
					pickUp('r');
					xd[2] += 0.8;
					right_arm->goToPose(xd, od);
					waitUntilFinish(right_arm);
					pickDown('r');
					arm->goToPose(beginXD, beginOD);
					waitUntilFinish(arm);
					right_arm->goToPose(beginXDR, beginODR);
					waitUntilFinish(right_arm);
				}
				else {
					arm->goToPose(xd,od);
					waitUntilFinish(arm);
					xd[0] += 0.04;
					arm->goToPose(xd, od);
					waitUntilFinish(arm);
					pickUp('l');
					xd[2] += 0.8;
					arm->goToPose(xd, od);
					waitUntilFinish(arm);
					pickDown('l');
					arm->goToPose(beginXD, beginOD);
					waitUntilFinish(arm);
					right_arm->goToPose(beginXDR, beginODR);
					waitUntilFinish(right_arm);
				}
			}
		}
    }
    
    void pickDown(char ch) {
		int lr;
		if(ch == 'l')
			lr = 0;
		else if(ch == 'r')
			lr = 1;
			
		int joints = 0;
		armp[lr]->getAxes(&joints);
		
		int i;
		for(i = 10; i < joints; i++) {
			armp[lr]->setRefSpeed(i, 10);
			armp[lr]->setRefAcceleration(i, 50);
		}
		armp[lr]->positionMove(7, 5);
		for(i = 10; i < joints; i++) {
			armp[lr]->positionMove(i, 10);
		}
		waitUntilFinish(armp[lr]);
	}
    
    void pickUp(char ch) {
		int lr;
		if(ch == 'l')
			lr = 0;
		else if(ch == 'r')
			lr = 1;	
		
		int joints = 0;
		armp[lr]->getAxes(&joints);
		
		int i;
		for(i = 10; i < joints; i++) {
			armp[lr]->setRefSpeed(i, 10);
			armp[lr]->setRefAcceleration(i, 50);
		}
		armp[lr]->setRefSpeed(5, 10);
		armp[lr]->setRefAcceleration(5, 50);
		
		disconnectCartesian('l');
		disconnectCartesian('r');
		armp[lr]->positionMove(5, -40);
		armp[lr]->positionMove(7, 15);
		waitUntilFinish(armp[lr]);
		connectCartesian('l');
		connectCartesian('r');
		
		if(ch == 'l') {
			arm->getPose(xd, od);
			xd[0] -= 0.02;
			xd[2] -= 0.06;
			arm->goToPose(xd, od);
			Time::delay(3);
			arm->stopControl();
		}
		else if(ch == 'r') {
			right_arm->getPose(xd, od);
			xd[0] -= 0.02;
			xd[2] -= 0.06;
			right_arm->goToPose(xd, od);
			Time::delay(3);
			right_arm->stopControl();
		}
		
		disconnectCartesian('l');
		disconnectCartesian('r');
		
		for(i = 10; i < joints; i++) {
			armp[lr]->positionMove(i, 80);
		}
		Time::delay(2);
		
		connectCartesian('l');
		connectCartesian('r');
	}

    double norm(const Vector &v)
    {
        return sqrt(dot(v,v));
    }

    void limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;
        
        arm->getLimits(axis,&min,&max);
        arm->setLimits(axis,min,MAX_TORSO_PITCH);
    }
};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new CtrlThread(CTRL_THREAD_PER);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main()
{   
    // we need to initialize the drivers list 
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    CtrlModule mod;

    ResourceFinder rf;
    return mod.runModule(rf);
}
