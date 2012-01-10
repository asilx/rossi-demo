#include <stdio.h>
#include <iostream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#define JOINT_0_MAX (25)
#define JOINT_0_MIN (-35)
#define JOINT_2_MAX (50)
#define JOINT_2_MIN (-50)

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

/* Controls whether given command to iCub in simulator is completed or not. 
 * If it is in process, then it waits until completed
 */
void
waitUntilFinish (IPositionControl* pos)
{
  bool done = false;

  while (!done)
  {
    Time::delay (0.001);
    pos->checkMotionDone (&done);
  }
}

int
main ()
{
  Network yarp; // yarp network
  BufferedPort<Vector> inputPort; // to get the position of object from "find.cpp"
  BufferedPort<Vector> finished; // to send signal to "operate.cpp" to indicate that focusing is finished
  finished.open ("/target/finished"); // opening port
  inputPort.open ("/target/in"); // opening port

  // connecting ports of "find.cpp" finished and "search.cpp" inputPort respectively
  Network::connect ("/target/out", "/target/in");

  // for head motions
  Property head;
  head.put ("device", "remote_controlboard");
  head.put ("local", "/client/search");
  head.put ("remote", "/icubSim/head");

  PolyDriver robotHead (head);

  // controls whether valid or not
  if (!robotHead.isValid ())
  {
    cout << "Connection cannot be established with head!" << endl;
    return (-1);
  }

  IPositionControl *pos;
  IVelocityControl *vel;
  IEncoders *ienc;
  bool ok = robotHead.view (pos) && robotHead.view (ienc) && robotHead.view (vel);

  if (!ok)
  {
    cout << "Cannot get interface to robot head!" << endl;
    robotHead.close ();
    return (-1);
  }

  int joints = 0;
  pos->getAxes (&joints);
  Vector command;
  Vector acc;
  Vector speed;
  Vector setpoints;
  command.resize (joints);
  speed.resize (joints);
  acc.resize (joints);
  setpoints.resize (joints);

  int i;
  // setting the speeds, accelerations and home position of head
  // ######################################################
  for (i = 0; i < joints; i++)
    speed[i] = 20;
  pos->setRefSpeeds (speed.data ());

  for (i = 0; i < joints; i++)
    acc[i] = 50;
  pos->setRefAccelerations (acc.data ());

  for (i = 0; i < joints; i++)
    command[i] = 0;

  //move HOME
  pos->positionMove (command.data ());
  waitUntilFinish (pos);
  // ######################################################

  double x = 0;
  double y = 0;

  int control = 0;

  while (true)
  {
    while (true)
    {
      Vector& bl = finished.prepare ();
      bl.resize (2);
      Vector *target = inputPort.read ();

      /* if iCub cannot find any object at its view angle, then
       * it randomly moves its head to find an object
       */
      if ((*target)[2] == 0 && control == 0)
      {
        bl[0] = 0;

        x = rand () % 50 - 35;
        y = rand () % 100 - 50;

        command[0] = x;
        command[2] = y;
        pos->positionMove (command.data ());
        waitUntilFinish (pos);
      }

      /* if iCub can find an object at its view angle, it
       * turns its head to the object until an object is at origin point
       * of its view. If this is not possible due to the motion ability,
       * then it breaks the while loop to allow the eye optimization*/
      else if ((*target)[2] == 1)
      {
        control = 1;
        if ((*target)[0] > 161)
          y -= 0.5;
        if ((*target)[0] < 159)
          y += 0.5;
        if ((*target)[1] > 121)
          x -= 0.5;
        if ((*target)[1] < 119)
          x += 0.5;

        if (((*target)[0] < 161 && (*target)[0] > 159) && ((*target)[1] < 121 && (*target)[1] > 119))
        {
          cout << "Head Optimization is COMPLETED!" << endl;
          break;
        }

        if ((x < JOINT_0_MAX) && (x > JOINT_0_MIN) && (y < JOINT_2_MAX) && (y > JOINT_2_MIN))
        {
          command[0] = x;
          command[2] = y;
          pos->positionMove (command.data ());
          waitUntilFinish (pos);
        }
        else if ((x >= JOINT_0_MAX) && (x <= JOINT_0_MIN) && (y >= JOINT_2_MAX) && (y <= JOINT_2_MIN))
        {
          cout << "Head Optimization is COMPLETED!" << endl;
          break;
        }
        else
          break;
      }
      finished.write ();
    }

    /* iCub turns its eyes to the object until an object is at origin point
     * of its view.*/
    if (control == 1)
    {
      Vector& bl = finished.prepare ();
      double prevX, prevY;
      bl.resize (2);
      bl[0] = 1;
      while (true)
      {
        Vector *target = inputPort.read ();
        if ((*target)[2] != 0)
        {
          double x = (*target)[0];
          double y = (*target)[1];
          double conf = (*target)[2];
          if ((x < 161 && x > 159) && (y < 121 && y > 119))
          {
            cout << "Eye Optimization is COMPLETED!" << endl;
            break;
          }
          else if (prevX == x && prevY == y)
            break;
          x -= 320 / 2;
          y -= 240 / 2;

          double vx = x * 0.1;
          double vy = -y * 0.1;

          for (int i = 0; i < joints; i++)
          {
            setpoints[i] = 0;
          }

          if (conf > 0.5)
          {
            setpoints[3] = vy;
            setpoints[4] = vx;
          }
          else
          {
            setpoints[3] = 0;
            setpoints[4] = 0;
          }
          vel->velocityMove (setpoints.data ());
          prevX = x;
          prevY = y;
        }
        else
          break;
      }
      finished.write ();
      control = 0;
    }
  }

  robotHead.close ();
  return (0);
}
