#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <iCub/perception/models.h>
#include <iCub/action/actionPrimitives.h>

#include <iCub/ctrl/math.h>  //for dcm2axis function
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <deque>

#define USE_LEFT    0
#define USE_RIGHT   1

#define AFFACTIONPRIMITIVESLAYER    ActionPrimitivesLayer1

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::perception;
using namespace iCub::action;

const double PI = 3.14159265;

Bottle bot_l_hand;
Bottle bot_r_hand;

class DataPort : public BufferedPort<Bottle>
{
  virtual void
  onRead (Bottle& b)
  {
    // process data in b

    size_t found;
    string left = "/left/in";
    found = this->getName ().find (left.c_str ());
    //if it belongs to /left/in, update bot_l_hand
    if (found != string::npos)
    {
      bot_l_hand = b;
      //			std::cout
      //					<<bot_l_hand.get(0).asDouble()<<" "
      //					<<bot_l_hand.get(1).asDouble()<<" "
      //					<<bot_l_hand.get(2).asDouble()<<" "<<std::endl;
    }
    //if it belongs to /right/in, update bot_r_hand
    else
      bot_l_hand = b;
  }
};

class ExampleModule : public RFModule
{
protected:
  AFFACTIONPRIMITIVESLAYER *action;
  BufferedPort<Bottle> inPort;
  DataPort l_hand_pose_in_port;
  //	DataPort r_hand_pose_in_port;
  Port rpcPort;

  Vector graspOrien;
  Vector graspDisp;
  Vector dOffs;
  Vector dLift;
  Vector home_x;
  Vector hand_x;

  bool openPorts;
  bool firstRun;

public:
  ExampleModule ()
  {
    graspOrien.resize (4);
    graspDisp.resize (4);
    dOffs.resize (3);
    dLift.resize (3);
    home_x.resize (3);

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
  }

  void
  getArmDependentOptions (Bottle &b, Vector &_gOrien, Vector &_gDisp, Vector &_dOffs, Vector &_dLift, Vector &_home_x)
  {
    if (Bottle *pB=b.find("grasp_orientation").asList())
    {
      int sz = pB->size ();
      int len = _gOrien.length ();
      int l = len < sz ? len : sz;

      for (int i = 0; i < l; i++)
        _gOrien[i] = pB->get (i).asDouble ();
    }

    if (Bottle *pB=b.find("grasp_displacement").asList())
    {
      int sz = pB->size ();
      int len = _gDisp.length ();
      int l = len < sz ? len : sz;

      for (int i = 0; i < l; i++)
        _gDisp[i] = pB->get (i).asDouble ();
    }

    if (Bottle *pB=b.find("systematic_error_displacement").asList())
    {
      int sz = pB->size ();
      int len = _dOffs.length ();
      int l = len < sz ? len : sz;

      for (int i = 0; i < l; i++)
        _dOffs[i] = pB->get (i).asDouble ();
    }

    if (Bottle *pB=b.find("lifting_displacement").asList())
    {
      int sz = pB->size ();
      int len = _dLift.length ();
      int l = len < sz ? len : sz;

      for (int i = 0; i < l; i++)
        _dLift[i] = pB->get (i).asDouble ();
    }

    if (Bottle *pB=b.find("home_position").asList())
    {
      int sz = pB->size ();
      int len = _home_x.length ();
      int l = len < sz ? len : sz;

      for (int i = 0; i < l; i++)
        _home_x[i] = pB->get (i).asDouble ();
    }
  }

  virtual bool
  configure (ResourceFinder &rf)
  {
    string name = rf.find ("name").asString ().c_str ();
    setName (name.c_str ());

    string partUsed = rf.find ("part").asString ().c_str ();
    if ((partUsed != "left_arm") && (partUsed != "right_arm"))
    {
      cout << "Invalid part requested!" << endl;
      return false;
    }

    Property config;
    config.fromConfigFile (rf.findFile ("from").c_str ());
    Bottle &bGeneral = config.findGroup ("general");
    if (bGeneral.isNull ())
    {
      cout << "Error: group general is missing!" << endl;
      return false;
    }

    // parsing general config options
    Property option;
    for (int i = 1; i < bGeneral.size (); i++)
    {
      Bottle *pB = bGeneral.get (i).asList ();
      if (pB->size () == 2)
        option.put (pB->get (0).asString ().c_str (), pB->get (1));
      else
      {
        cout << "Error: invalid option!" << endl;
        return false;
      }
    }

    option.put ("local", name.c_str ());
    option.put ("part", rf.find ("part").asString ().c_str ());
    option.put ("grasp_model_type", rf.find ("grasp_model_type").asString ().c_str ());
    option.put ("grasp_model_file", rf.findFile ("grasp_model_file").c_str ());
    option.put ("hand_sequences_file", rf.findFile ("hand_sequences_file").c_str ());

    // parsing arm dependent config options
    Bottle &bArm = config.findGroup ("arm_dependent");
    getArmDependentOptions (bArm, graspOrien, graspDisp, dOffs, dLift, home_x);

    cout << "***** Instantiating primitives for " << partUsed << endl;
    action = new AFFACTIONPRIMITIVESLAYER (option);
    if (!action->isValid ())
    {
      delete action;
      return false;
    }

    deque<string> q = action->getHandSeqList ();
    cout << "***** List of available hand sequence keys:" << endl;
    for (size_t i = 0; i < q.size (); i++)
      cout << q[i] << endl;

    string fwslash = "/";
    inPort.open ((fwslash + name + "/in").c_str ());
    rpcPort.open ((fwslash + name + "/rpc").c_str ());
    attach (rpcPort);

    l_hand_pose_in_port.useCallback ();
    l_hand_pose_in_port.open ((fwslash + name + "/left/in").c_str ());
    //		r_hand_pose_in_port.useCallback();
    //		r_hand_pose_in_port.open((fwslash + name + "/right/in").c_str());

    Network::connect ("/icubSim/cartesianController/left_arm/state:o", l_hand_pose_in_port.getName ());
    std::cout << "*********" << std::endl;
    std::cout << Network::isConnected ("/icubSim/cartesianController/left_arm/state:o", l_hand_pose_in_port.getName ())
        << std::endl;
    std::cout << "---------" << std::endl;

    openPorts = true;

    // check whether the grasp model is calibrated,
    // otherwise calibrate it and save the results
    Model *model;
    action->getGraspModel (model);
    if (!model->isCalibrated ())
    {
      Property prop ("(finger all)");
      model->calibrate (prop);

      ofstream fout;
      fout.open (option.find ("grasp_model_file").asString ().c_str ());
      model->toStream (fout);
      fout.close ();
    }

    return true;
  }

  virtual bool
  close ()
  {
    if (action != NULL)
      delete action;

    if (openPorts)
    {
      inPort.close ();
      rpcPort.close ();
    }

    return true;
  }

  virtual double
  getPeriod ()
  {
    return 0.1;
  }

  void
  init ()
  {
    bool f;

    action->pushAction (home_x, "open_hand");
    action->checkActionsDone (f, true);
    action->enableArmWaving (home_x);
  }

  // we don't need a thread since the actions library already
  // incapsulates one inside dealing with all the tight time constraints

  Vector
  getNormalVector (Vector vec_angle_rep)
  {
    Matrix R = iCub::ctrl::axis2dcm (vec_angle_rep);

    //    for (uint i = 0; i < R.rows (); i++)
    //    {
    //      for (uint j = 0; j < R.cols (); j++)
    //      {
    //        std::cout << R[i][j] << " ";
    //      }
    //      std::cout << std::endl;
    //    }

    Vector v (3);
    v[0] = R[0][2];
    v[1] = R[1][2];
    v[2] = R[2][2];
    return v;
  }

  Vector
  normal2VectorAngle (const Vector& hand_normal)
  {
    double z_theta_hand = atan2 (hand_normal[1], hand_normal[0]);
    std::cout << "theta: " << z_theta_hand << std::endl;
    //    Vector oz (4);
    Vector ox (4);
    Vector oz_final (4);

    //    oz[0] = 0.0;
    //    oz[1] = 0.0;
    //    oz[2] = 1.0;
    //    oz[3] = PI;

    ox[0] = 1.0;
    ox[1] = 0.0;
    ox[2] = 0.0;
    ox[3] = -PI / 2;

    oz_final[0] = 0.0;
    oz_final[1] = 0.0;
    oz_final[2] = 1.0;
    oz_final[3] = z_theta_hand - PI / 2;

    //    Matrix Rz = iCub::ctrl::axis2dcm (oz); // from axis/angle to rotation matrix notation
    Matrix Rx = iCub::ctrl::axis2dcm (ox);
    Matrix Rz_final = iCub::ctrl::axis2dcm (oz_final);

    Matrix R = Rz_final * Rx;
    Vector vec_ang = iCub::ctrl::dcm2axis (R);
    return vec_ang;
  }

  Vector
  angleXZToVectorAngle (const double x_ang, const double z_ang)
  {
    Vector oz (4);
    Vector ox (4);

    oz[0] = 0.0;
    oz[1] = 0.0;
    oz[2] = 1.0;
    oz[3] = z_ang / 180 * PI;

    ox[0] = 1.0;
    ox[1] = 0.0;
    ox[2] = 0.0;
    ox[3] = x_ang / 180 * PI;

    Matrix Rz = iCub::ctrl::axis2dcm (oz); // from axis/angle to rotation matrix notation
    Matrix Rx = iCub::ctrl::axis2dcm (ox);

    Matrix R = Rz * Rx;
    Vector poi_orient = iCub::ctrl::dcm2axis (R); // from rotation matrix back to the axis/angle notation

    //    std::cout << poi_orient[0] << " " << poi_orient[1] << " " << poi_orient[2] << std::endl;

    return poi_orient;
  }

  Vector
  world2rootRef (const double x, const double y, const double z)
  {

    //    Matrix m;
    //    m.resize(4,4);
    //    m[0][0]= 0;m[0][1]=-1;m[0][2]=0;m[0][3]=0;
    //    m[1][0]= 0;m[1][1]= 0;m[1][2]=1;m[1][3]=0.584;
    //    m[2][0]=-1;m[2][1]= 0;m[2][2]=0;m[2][3]=-0.04;
    //    m[3][0]= 0;m[3][1]= 0;m[3][2]=0;m[3][3]=1;
    //
    //    Matrix v_w(4,1);
    //    v_w[0][0]=x;
    //    v_w[1][0]=y;
    //    v_w[2][0]=z;
    //
    //    Matrix v_r(4,1);
    //    v_r = v_w.transposed() * m;
    //
    Vector res (3);
    //    res[0]=v_r.transposed()[0][0];
    //    res[1]=v_r.transposed()[1][0];
    //    res[2]=v_r.transposed()[2][0];

    res[0] = -z - 0.04;
    res[1] = -x;
    res[2] = y - 0.5484;

    std::cout << res[0] << " " << res[1] << " " << res[2] << std::endl;

    return res;
  }

  Vector
  world2rootRef (const Vector& v_in_world)
  {

    Vector v_in_root (3);

    v_in_root[0] = -v_in_world[2] - 0.04;
    v_in_root[1] = -v_in_world[0];
    v_in_root[2] = v_in_world[1] - 0.5484;

    return v_in_root;
  }

  //reach point-of-interest with an offset of poi_off and in the orientation of poi_orient to
  //shift this point in the same direction with an amount of poi_shift
  void
  push (const Vector& poi_x, const Vector& push_dir, const double poi_offset, const double poi_shift)
  {
    bool f;

    //just consider for now that pushes are done on the x-y plane, specifically on the table
    Vector hand_normal (3);
    Vector poi_off (3);
    poi_off[0]=0.0;
    poi_off[1]=0.0;
    poi_off[2]=0.0;

    //decide on how to interact with the object
    double push_dir_angle = atan2 (push_dir[1], push_dir[0]);
    if (push_dir_angle < 0)
      push_dir_angle += 2 * PI;
    std::cout << "push angle: " << push_dir_angle * 180 / PI << std::endl;
    double hand_nor_angle = 0;
    string hand_key;

    if (PI / 4 <= push_dir_angle && push_dir_angle < 3 * PI / 4)
    {
      hand_nor_angle = push_dir_angle + PI;
      hand_key = "karate_hand";
      std::cout << hand_key << std::endl;
    }
    else if (3 * PI / 4 <= push_dir_angle && push_dir_angle < 5 * PI / 4)
    {
      hand_nor_angle = push_dir_angle + PI / 2;
//      hand_key = "close_hand";
      hand_key = "fist_hand";
      std::cout << hand_key << std::endl;
    }
    else if (5 * PI / 4 <= push_dir_angle && push_dir_angle < 7 * PI / 4)
    {
      hand_nor_angle = push_dir_angle;
      hand_key = "karate_hand";
      std::cout << hand_key << std::endl;
    }
    else
    {
      hand_nor_angle = push_dir_angle - PI / 2.0;
      poi_off[1]= - 0.04;
      poi_off[0] = push_dir[0] * (poi_offset);
      hand_key = "perpendicular_hand";
      std::cout << hand_key << std::endl;
    }

    hand_normal[0] = cos (hand_nor_angle);
    hand_normal[1] = sin (hand_nor_angle);
    hand_normal[2] = 0;

    poi_off[0] += push_dir[0] * (-poi_offset);// x component of the direction vector * reach offset
    poi_off[1] += push_dir[1] * (-poi_offset);// y component of the direction vector * reach offset
    poi_off[2] = push_dir[2] * (-poi_offset);// z component of the direction vector * reach offset

    //Find the vector-angle representation of this normal
    Vector vec_angle = normal2VectorAngle (hand_normal);

    //    std::cout << poi_x[0] << " " << poi_x[1] << " " << poi_x[2] << std::endl;
    //    std::cout << poi_off[0] << " " << poi_off[1] << " " << poi_off[2] << std::endl;
    //    action->touch (poi_x, vec_angle, poi_off);

    action->pushAction (poi_x + poi_off, vec_angle, hand_key);
    action->checkActionsDone (f, true);

    Vector poi_sh (3);
    poi_sh[0] = push_dir[0] * poi_shift;// x component of the direction vector * reach offset
    poi_sh[1] = push_dir[1] * poi_shift;// y component of the direction vector * reach offset
    poi_sh[2] = push_dir[2] * poi_shift;// z component of the direction vector * reach offset

    action->pushAction (poi_x + poi_sh, vec_angle, hand_key);
    action->checkActionsDone (f, true);

  }

  virtual bool
  updateModule ()
  {
    if (firstRun)
    {
      init ();
      firstRun = false;
    }

    bool f;
    std::cout << "(1) push" << std::endl;
    std::cout << "(2) point" << std::endl;
    std::cout << "(3) basket" << std::endl;
    Bottle *b = inPort.read (); // blocking call
    switch (b->get (0).asInt ())
    {
      case 1:
        //left hand assumed
        std::cout << "<point of interest> <push direction>" << std::endl;
        b = inPort.read ();
        if (b->size () == 6)//3 for position 3 for direction
        {
          Vector poi (3);
          poi[0] = b->get (0).asDouble ();
          poi[1] = b->get (1).asDouble ();
          poi[2] = b->get (2).asDouble ();

          //          poi = world2rootRef(poi);
          poi = world2rootRef (poi[0], poi[1], poi[2]);

          Vector push_dir (3);
          push_dir[0] = b->get (3).asDouble ();
          push_dir[1] = b->get (4).asDouble ();
          push_dir[2] = b->get (5).asDouble ();

          double offset = 0.06;
          double shift = 0.10;
          push (poi, push_dir, offset, shift);//blocking operation
        }
        break;
      case 3:

      default:
        break;
    }

    action->pushAction ("open_hand");
    action->checkActionsDone (f, true);

    // go home :) (wait until it's done, since
    // we may use two arms that share the torso)
    action->pushAction (home_x);
    action->checkActionsDone (f, true);

    // let the hand wave a bit around home position
    // the waving will be disabled before commencing
    // a new action
    action->enableArmWaving (home_x);

    /*
     Vector xd (2);
     xd[0] = b->get (0).asDouble ();
     xd[1] = b->get (1).asDouble ();

     Vector poi_orient = angleXZToVectorAngle (xd[0], xd[1]);

     Vector xdes (3);
     xdes[0] = -0.25;
     xdes[1] = -0.15;
     xdes[2] = 0.15;

     bool f;
     action->pushAction (xdes, poi_orient, "open_hand");
     action->checkActionsDone (f, true);

     Vector left_xc (3);
     left_xc[0] = bot_l_hand.get (0).asDouble ();
     left_xc[1] = bot_l_hand.get (1).asDouble ();
     left_xc[2] = bot_l_hand.get (2).asDouble ();

     Vector left (4);
     left[0] = bot_l_hand.get (3).asDouble ();
     left[1] = bot_l_hand.get (4).asDouble ();
     left[2] = bot_l_hand.get (5).asDouble ();
     left[3] = bot_l_hand.get (6).asDouble ();

     Vector hand_normal = getNormalVector (left);
     std::cout << hand_normal[0] << " " << hand_normal[1] << " " << hand_normal[2] << " " << std::endl;

     //    xd[2] = b->get (2).asDouble ();

     //    xd = world2rootRef (xd);

     //    bool f;

     //    double z_ang = PI;
     //    double x_ang = -PI / 2;


     //    double poi_offset = -0.08;
     //    double poi_shift = 0.20;


     //    push (xd, poi_orient, poi_offset, poi_shift);

     //    Vector left (4);
     //    left[0] = bot_l_hand.get (3).asDouble ();
     //    left[1] = bot_l_hand.get (4).asDouble ();
     //    left[2] = bot_l_hand.get (5).asDouble ();
     //    left[3] = bot_l_hand.get (6).asDouble ();
     //
     //    Vector v = getNormalVector (left);

     //    std::cout<<left[0]<<" "<<left[1]<<" "<<left[2]<<" "<<left[3]<<std::endl;


     /*
     // get a target object position from a YARP port
     //    std::cout << "push left left_hand delta_x hand_orientation" << std::endl;
     std::cout << "choose hand_orientation" << std::endl;
     Bottle *b = inPort.read (); // blocking call
     Vector xd (3);
     //		xd[0] = b->get(1).asDouble();
     //		xd[1] = b->get(2).asDouble();
     //		xd[2] = b->get(3).asDouble();

     int dir = b->get (0).asInt ();
     bool f;

     Vector left_xc (3);
     left_xc[0] = bot_l_hand.get (0).asDouble ();
     left_xc[1] = bot_l_hand.get (1).asDouble ();
     left_xc[2] = bot_l_hand.get (2).asDouble ();

     Vector delta_x (3);
     delta_x[0] = 0;
     delta_x[1] = 0;
     delta_x[2] = 0;

     graspOrien[0] = b->get (1).asDouble ();
     graspOrien[1] = b->get (2).asDouble ();
     graspOrien[2] = b->get (3).asDouble ();
     graspOrien[3] = b->get (4).asDouble () / 180 * PI;

     Vector oz (4);
     Vector ox (4);

     Matrix Rz; // from axis/angle to rotation matrix notation
     Matrix Rx;

     Matrix R;
     Vector o;

     switch (dir)
     {
     case 0:
     //hand orientation
     //        xd[0]= -0.1;
     //        xd[1]= -0.45;
     //        xd[2]= 0.2;

     xd[0] = -0.3;
     xd[1] = -0.15;
     xd[2] = 0.2;

     oz[0] = 0.0;
     oz[1] = 0.0;
     oz[2] = 1.0;
     oz[3] = PI;

     ox[0] = 1.0;
     ox[1] = 0.0;
     ox[2] = 0.0;
     ox[3] = -PI / 2.0;

     Rz = iCub::ctrl::axis2dcm (oz); // from axis/angle to rotation matrix notation
     Rx = iCub::ctrl::axis2dcm (ox);

     R = Rz * Rx;
     o = iCub::ctrl::dcm2axis (R); // from rotation matrix back to the axis/angle notation

     action->pushAction (xd, o, "karate_hand");
     action->checkActionsDone (f, true);
     break;
     case 1:
     //push right
     delta_x[1] = 0.2;
     xd = left_xc + delta_x;
     action->pushAction (xd, graspOrien);
     action->checkActionsDone (f, true);
     break;

     case 2:
     //push fwd
     delta_x[0] = -0.2;
     xd = left_xc + delta_x;
     action->pushAction (xd, graspOrien);
     action->checkActionsDone (f, true);
     break;

     case 3:
     //push left
     delta_x[1] = -0.2;
     xd = left_xc + delta_x;
     action->pushAction (xd, graspOrien);
     action->checkActionsDone (f, true);
     break;

     case 4:
     //push bwd
     delta_x[0] = 0.2;
     xd = left_xc + delta_x;
     action->pushAction (xd, graspOrien);
     action->checkActionsDone (f, true);
     break;
     case 5:
     action->pushAction ("close_hand");
     action->checkActionsDone (f, true);
     break;
     case 6:
     action->pushAction ("open_hand");
     action->checkActionsDone (f, true);
     break;
     case 7:
     action->pushAction ("point_hand");
     action->checkActionsDone (f, true);
     break;
     default:
     break;
     }
     */
    //    std::cout << xd[0] << " " << xd[1] << " " << xd[2] << std::endl;

    //    action->pushAction ("open_hand");
    //    action->checkActionsDone (f, true);
    //

    /*
     if (b!=NULL)
     {
     Vector xd(3);
     bool f;

     xd[0]=b->get(0).asDouble();
     xd[1]=b->get(1).asDouble();
     xd[2]=b->get(2).asDouble();

     // apply systematic offset
     // due to uncalibrated kinematic
     xd=xd+dOffs;
     }
     */
    return true;
  }

  //	virtual bool updateModule() {
  //		//update left hand pose
  //		//update right hand pose
  //
  //		// do it only once
  //		if (firstRun) {
  //			init();
  //			firstRun = false;
  //		}
  //
  //		// get a target object position from a YARP port
  //		std::cout << "which way you wanna push? 1-2-3-4, r-fwd-l-bwd-circle"
  //				<< std::endl;
  //		Bottle *b = inPort.read(); // blocking call
  //		Vector xd(3);
  //		//		xd[0] = b->get(1).asDouble();
  //		//		xd[1] = b->get(2).asDouble();
  //		//		xd[2] = b->get(3).asDouble();
  //
  //		int dir = b->get(0).asInt();
  //		bool f;
  //
  //		Vector left_xc(3);
  //		left_xc[0] = bot_l_hand.get(0).asDouble();
  //		left_xc[1] = bot_l_hand.get(1).asDouble();
  //		left_xc[2] = bot_l_hand.get(2).asDouble();
  //
  //		Vector delta_x(3);
  //		delta_x[0] = 0;
  //		delta_x[1] = 0;
  //		delta_x[2] = 0;
  //
  //		graspOrien[0] = 1;
  //		graspOrien[1] = 0;
  //		graspOrien[2] = 0;
  //		graspOrien[3] = 1.57;
  //
  //		switch (dir) {
  //		case 1:
  //			//push right
  //			delta_x[1] = 0.2;
  //			xd = left_xc + delta_x;
  //			action->pushAction(xd, graspOrien);
  //			action->checkActionsDone(f, true);
  //			break;
  //
  //		case 2:
  //			//push fwd
  //			delta_x[0] = -0.2;
  //			xd = left_xc + delta_x;
  //			action->pushAction(xd, graspOrien);
  //			action->checkActionsDone(f, true);
  //			break;
  //
  //		case 3:
  //			//push left
  //			delta_x[1] = -0.2;
  //			xd = left_xc + delta_x;
  //			action->pushAction(xd, graspOrien);
  //			action->checkActionsDone(f, true);
  //			break;
  //
  //		case 4:
  //			//push bwd
  //			delta_x[0] = 0.2;
  //			xd = left_xc + delta_x;
  //			action->pushAction(xd, graspOrien);
  //			action->checkActionsDone(f, true);
  //			break;
  //		case 5:
  //			action->pushAction("close_hand");
  //			action->checkActionsDone(f, true);
  //			break;
  //		case 6:
  //			action->pushAction("open_hand");
  //			action->checkActionsDone(f, true);
  //			break;
  //		case 7:
  //			action->pushAction("point_hand");
  //			action->checkActionsDone(f, true);
  //			break;
  //		default:
  //			break;
  //		}
  //
  //		std::cout << xd[0] << " " << xd[1] << " " << xd[2] << std::endl;
  //
  //		action->pushAction("open_hand");
  //		action->checkActionsDone(f, true);
  //
  //		// go home :) (wait until it's done, since
  //		// we may use two arms that share the torso)
  //		action->pushAction(home_x);
  //		action->checkActionsDone(f, true);
  //
  //		// let the hand wave a bit around home position
  //		// the waving will be disabled before commencing
  //		// a new action
  //		action->enableArmWaving(home_x);
  //
  //		/*
  //		 if (b!=NULL)
  //		 {
  //		 Vector xd(3);
  //		 bool f;
  //
  //		 xd[0]=b->get(0).asDouble();
  //		 xd[1]=b->get(1).asDouble();
  //		 xd[2]=b->get(2).asDouble();
  //
  //		 // apply systematic offset
  //		 // due to uncalibrated kinematic
  //		 xd=xd+dOffs;
  //		 }
  //		 */
  //		return true;
  //	}

  bool
  interruptModule ()
  {
    // since a call to checkActionsDone() blocks
    // the execution until it's done, we need to
    // take control and exit from the waiting state
    action->syncCheckInterrupt (true);

    inPort.interrupt ();
    rpcPort.interrupt ();
    l_hand_pose_in_port.interrupt ();
    //		r_hand_pose_in_port.interrupt();

    return true;
  }
};

int
main (int argc, char *argv[])
{
  Network yarp;

  if (!yarp.checkNetwork ())
    return -1;

  YARP_REGISTER_DEVICES(icubmod)

  ResourceFinder rf;
  rf.setVerbose (true);
  rf.setDefaultContext ("actionPrimitivesExample/conf");
  rf.setDefaultConfigFile ("config.ini");
  rf.setDefault ("part", "left_arm");
  rf.setDefault ("grasp_model_type", "springy");
  rf.setDefault ("grasp_model_file", "grasp_model.ini");
  rf.setDefault ("hand_sequences_file", "hand_sequences.ini");
  rf.setDefault ("name", "actionPrimitivesMod");
  rf.configure ("ICUB_ROOT", argc, argv);

  ExampleModule mod;

  return mod.runModule (rf);
}

