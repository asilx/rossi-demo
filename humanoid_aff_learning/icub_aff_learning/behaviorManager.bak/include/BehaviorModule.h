#ifndef BEHAVIORMODULE_H_
#define BEHAVIORMODULE_H_

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <iCub/perception/models.h>
#include <iCub/action/actionPrimitives.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <deque>
#include <pthread.h>
#include <math.h>

//ros includes


#define PI 3.14159265

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::perception;
using namespace iCub::action;

class BehaviorModule : public RFModule
{
protected:
  ActionPrimitivesLayer2 *action;
  BufferedPort<Bottle> inPort;
  BufferedPort<Bottle> port_sim_rpc_out;
  BufferedPort<Bottle> port_simon_in;
  Port rpcPort;
  yarp::os::Semaphore menu_mutex;

  Property options_left, options_right;
  PolyDriver driver_left, driver_right;
  IPositionControl* pos_ctrl_left, *pos_ctrl_right;
  IEncoders* encoders_left, *encoders_right;

  Vector positions_left_cmd;
  Vector positions_left_enc;
  Vector positions_right_cmd;
  Vector positions_right_enc;

  Vector graspOrien;
  Vector graspDisp;
  Vector dOffs;
  Vector dLift;
  Vector home_x;

  bool openPorts;
  bool firstRun;
  bool sim;

public:
  BehaviorModule ();
  ~BehaviorModule ();

  void
  getArmDependentOptions (Bottle &b, Vector &_gOrien, Vector &_gDisp, Vector &_dOffs, Vector &_dLift, Vector &_home_x);

  virtual bool
  configure (ResourceFinder &rf);

  virtual bool
  close ();

  virtual double
  getPeriod ();

  void
  init ();

  Vector
  vectorAngle2Normal (Vector vec_angle_rep);

  Vector
  normal2VectorAngle (const Vector& hand_normal);

  Vector
  angleXZToVectorAngle (const double x_ang, const double z_ang);

  Vector
  angleXYZToVectorAngle (const double x_ang, const double y_ang, const double z_ang);

  //bb_pos and bb_dims specifies the bounding box properties of the object of interest
  //push_dir is the angle(in radians). 0 -> right, PI/2 -> forward, PI -> left, 3*PI/2 -> backward
  void
  push (const Vector& bb_center, const Vector& bb_dims, double push_dir_angle, const double poi_shift, bool spin =
      false);

  void
  lift (const double poi_shift);

  void
  hide (Vector bb_center, Vector bb_dims);

  void
  pointTo (Vector point);

  void
  release (Vector point, bool palm_upward = true);

  void
  grasp (Vector bb_center);

  void
  graspLiftAndRelease (Vector bb_center, Vector bb_target_center, const double shift, bool palm_upward = false);

  void
  home ();

  void
  tuckArms ();

  int
  voiceCommand (Vector bb_center, Vector bb_dims);

  int
  manualCommand (Vector bb_center, Vector bb_dims);

  //from top or sides (left, right, back)
  void
  fetch (Vector bb_center, Vector bb_dims, Vector hand_orient);

  void
  shake (const Vector dir, const double shake_off);

  void
  basket (Vector bb_center, Vector bb_dims);

  void
  testHandSequences ();

  void
  createObject (int op_type, Vector& bb_center);

  void
  deleteObject ();

  // we don't need a thread since the actions library already
  // incapsulates one inside dealing with all the tight time constraints
  virtual bool
  updateModule ();

  bool
  interruptModule ();
};

#endif /* BEHAVIORMODULE_H_ */
