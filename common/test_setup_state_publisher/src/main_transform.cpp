//ros includes
#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/JointState.h>

// yarp includes
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <pthread.h>

// standard includes
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

#define CHANNEL209 "Channel105"
#define CHANNEL210 "Channel106"
#define CHANNEL211 "Channel107"
#define CHANNEL212 "Channel108"

typedef struct ledVector
{
  ConstString channelName;
  Vector coordinates;

  ledVector ()
  {
    coordinates.resize (3);
  }
} LEDVector;

typedef struct transform
{
  Vector rpy;
  Vector offset;
  transform ()
  {
    rpy.resize (3);
    offset.resize (3);
  }
} Transform;

vector<LEDVector> vectorList;
Vector rpy (3);
Vector offset (3);
bool first_data_received = false;
bool new_data_received = false;
Transform prev_transform;
Transform avg_tranform;
tf::StampedTransform* avg_tf = NULL;
bool tf_calculated = false;

const int N_TRANSFORM = 10;

Vector
getCoordinatesByChannelName (const char* channelOfInterest)
{
  for (uint i = 0; i < vectorList.size (); i++)
    if (vectorList[i].channelName == channelOfInterest)
      return vectorList[i].coordinates;

  cerr << "Could not find the queried channel of interest, something wrong with the leds or mocap, EXITING!" << endl;
  exit (-1);
}

bool
getCameraLedInfo (const Bottle& b)
{

  int channelCount = b.get (0).asInt ();//get number of active leds
  vectorList.clear ();

  for (int i = 1; i < channelCount * 5; i = i + 5)
  {
    LEDVector led;
    led.channelName = b.get (i).asString ();//channel name

    //now convert led coordinates from KDL frame back to the iCub frame
    /*
     led.coordinates[0] = -b.get (i + 3).asDouble ();//KDL's "-z" -> icub's "x"
     led.coordinates[1] = -b.get (i + 1).asDouble ();//KDL's "-x" -> icub's "y"
     led.coordinates[2] = b.get (i + 2).asDouble ();//KDL's "y" -> icub's "z"
     */

    //convert vz to icub frame
    led.coordinates[0] = -b.get (i + 1).asDouble ();// VZ::-x -> icub::x
    led.coordinates[1] = b.get (i + 2).asDouble ();// VZ::y -> icub::y
    led.coordinates[2] = -b.get (i + 3).asDouble (); // VZ::-z -> icub::z

    if (fabs (led.coordinates[0]) < 0.001 && fabs (led.coordinates[1]) < 0.001 && fabs (led.coordinates[2]) < 0.001)
      return false;

    led.coordinates[0] += 0.38;
    led.coordinates[1] -= 0.095;
    led.coordinates[2] -= 0.0018;

    //    cout << led.channelName << " " << led.coordinates[0] << " " << led.coordinates[1] << " " << led.coordinates[2]<< endl;
    vectorList.push_back (led);
  }
  return true;
}

void
getTransformation (Vector& offset, Vector& rpy)
{
  Vector ch209 = getCoordinatesByChannelName (CHANNEL209);
  //  Vector ch210 = getCoordinatesByChannelName (CHANNEL210);
  Vector ch211 = getCoordinatesByChannelName (CHANNEL211);
  Vector ch212 = getCoordinatesByChannelName (CHANNEL212);

  //get camera casis coordinate system

  //get origin:
  Vector cam_casis_origin = ch211;
  offset = cam_casis_origin;

  //get unit x vector
  Vector cam_casis_x = ch212 - cam_casis_origin;
  double norm_cam_casis_x = norm (cam_casis_x);
  cam_casis_x = cam_casis_x / norm_cam_casis_x;

  //get unit xz plane vector
  //  Vector cam_casis_xz = (ch209 + ch210)/2 - cam_casis_origin;
  Vector cam_casis_xz = ch209 - cam_casis_origin;
  double norm_cam_casis_xz = norm (cam_casis_xz);
  cam_casis_xz = cam_casis_xz / norm_cam_casis_xz;

  //get unit y vector
  Vector cam_casis_y = cross (cam_casis_xz, cam_casis_x);
  double norm_cam_casis_y = norm (cam_casis_y);
  cam_casis_y = cam_casis_y / norm_cam_casis_y;

  //get unit z vector
  Vector cam_casis_z = cross (cam_casis_x, cam_casis_y);
  double norm_cam_casis_z = norm (cam_casis_z);
  cam_casis_z = cam_casis_z / norm_cam_casis_z;

  //find rpy component
  Matrix rot (3, 3);

  Vector xG (3);
  xG[0] = 1;
  xG[1] = 0;
  xG[2] = 0;
  Vector yG (3);
  yG[0] = 0;
  yG[1] = 1;
  yG[2] = 0;
  Vector zG (3);
  zG[0] = 0;
  zG[1] = 0;
  zG[2] = 1;

  Vector v (3);
  v[0] = dot (cam_casis_x, xG);
  v[1] = dot (cam_casis_x, yG);
  v[2] = dot (cam_casis_x, zG);
  rot.setCol (0, v);

  v[0] = dot (cam_casis_y, xG);
  v[1] = dot (cam_casis_y, yG);
  v[2] = dot (cam_casis_y, zG);
  rot.setCol (1, v);

  v[0] = dot (cam_casis_z, xG);
  v[1] = dot (cam_casis_z, yG);
  v[2] = dot (cam_casis_z, zG);
  rot.setCol (2, v);

  rpy[0] = atan2 (rot (2, 1), rot (2, 2)); // r
  rpy[2] = atan2 (rot (1, 0), rot (0, 0)); // y
  rpy[1] = atan2 (-rot (2, 0), cos (rpy[2]) * rot (0, 0) + sin (rpy[2]) * rot (1, 0)); // p
}

tf::Transform
getTF (const Vector& offset, const Vector& rpy)
{
  tf::Transform icub_to_cam_top;
  icub_to_cam_top.setOrigin (tf::Vector3 (offset[0], offset[1], offset[2]));
  icub_to_cam_top.setRotation (tf::createQuaternionFromRPY (rpy[0], rpy[1], rpy[2]));

  tf::Transform cam_top_to_cam_focus;
  //from led103 towards led104, x-direction, camera width=66mm, consider led diameter 5mm
  //from led103 towards led101, z-direction, camera depth 76mm, consider led diameter 5mm
  cam_top_to_cam_focus.setOrigin (tf::Vector3 (0.030, 0.030, 0.079));
  cam_top_to_cam_focus.setRotation (tf::createQuaternionFromRPY (0, 0, 0));

  //return the tf from icub to cam focus
  icub_to_cam_top.mult (icub_to_cam_top, cam_top_to_cam_focus);
  return icub_to_cam_top;
}

tf::Transform
calcTF (Vector& offset, Vector& rpy)
{
  getTransformation (offset, rpy);

  return getTF (offset, rpy);
}

//mocap data received callback
class DataPort : public BufferedPort<Bottle>
{
  virtual void
  onRead (Bottle& b)
  {
    static int data_count = 0;
    // process data in b
    bool safe_tf = getCameraLedInfo (b);
    if (data_count < N_TRANSFORM)
    {
      if (safe_tf)
      {
        calcTF (offset, rpy);
        for (uint8_t i = 0; i < offset.size (); i++)
          avg_tranform.offset[i] += offset[i] / N_TRANSFORM;
        for (uint8_t i = 0; i < rpy.size (); i++)
          avg_tranform.rpy[i] += rpy[i] / N_TRANSFORM;
        data_count++;
      }
    }
    else
    {
      std::cout << "calculating average transform" << std::endl;
      avg_tf = new tf::StampedTransform (getTF (avg_tranform.offset, avg_tranform.rpy), ros::Time::now (),
                                         "/base_link", "/swissranger_link");
      this->close (); //close this connection
    }
  }
/*
  virtual void
  onRead (Bottle& b)
  {
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    new_data_received = true;

    // process data in b
    bool safe_tf = getCameraLedInfo (b);
    if (!safe_tf)
    {
      if (first_data_received)//if first data is corrupted don't do anything

      {
        std::cout << "not safe to transform" << std::endl;
        //first parent frame, then child frame
        br.sendTransform (tf::StampedTransform (getTF (prev_transform.offset, prev_transform.rpy), ros::Time::now (),
                                                "/base_link", "/swissranger_link"));
      }
      else
        return;
    }
    else
    {
      //first parent frame, then child frame
      br.sendTransform (tf::StampedTransform (calcTF (offset, rpy), ros::Time::now (), "/base_link",
                                              "/swissranger_link"));
      prev_transform.offset = offset;
      prev_transform.rpy = rpy;
    }

    if (!first_data_received)
      first_data_received = true;
  }
  */
};

int
main (int argc, char* argv[])
{
  std::cout << "haydeee" << std::endl;
  //ros initializations
  ros::init (argc, argv, "mocap");
  ros::NodeHandle n;

  //yarp initializations
  Network::init ();
  DataPort vzInPort;
  vzInPort.useCallback ();
  vzInPort.open ("/i:vzListen");
  Network::connect ("/vzRawPout", "/i:vzListen");

  std::cout << "waiting for mocap to be connected" << std::endl;
  while (!Network::isConnected ("/vzRawPout", "/i:vzListen") && n.ok ())
  {
  }
  std::cout << "mocap is connected, ready for led data..." << std::endl;

  static tf::TransformBroadcaster br;

  ros::Rate r (50);
  while (n.ok ())
  {
    if (avg_tf != NULL)
    {
      br.sendTransform (tf::StampedTransform (*avg_tf, ros::Time::now (), "/base_link", "/swissranger_link"));
    }
    r.sleep ();
    ros::spinOnce ();
  }
  Network::fini ();
  delete avg_tf;

  return 0;
}
