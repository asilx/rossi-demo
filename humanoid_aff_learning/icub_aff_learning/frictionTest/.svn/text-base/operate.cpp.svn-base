#include <stdio.h>
#include <iostream>
#include <string.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

/* WorldObj class is sufficient to hold the position and the radius or
 * length, width, height information of an object
 */

class WorldObj
{
private:
  double x, y, z;
  double radius;
  double length;
  double width;
  double height;
public:
  WorldObj ()
  {
    radius = -1;
    length = -1;
    width = -1;
    height = -1;
    x = y = z = -1;
  }
  ~WorldObj ()
  {
  }

  /* All required get functions to obtain the corresponding information */
  double
  getX ()
  {
    return this->x;
  }
  double
  getY ()
  {
    return this->y;
  }
  double
  getZ ()
  {
    return this->z;
  }
  double
  getRadius ()
  {
    return this->radius;
  }
  double
  getLength ()
  {
    return this->length;
  }
  double
  getWidth ()
  {
    return this->width;
  }
  double
  getHeight ()
  {
    return this->height;
  }

  // ##################################################

  /* All required add functions to set the corresponding information */
  void
  addRadius (double radius)
  {
    this->radius = radius;
  }
  void
  addPositionCoordinates (double x, double y, double z)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  void
  addLWH (double length, double width, double height)
  {
    this->length = length;
    this->width = width;
    this->height = height;
  }
  // ##################################################

  /* Prints the information of an object with respect to its type (sphere or box) */
  void
  printObjects ()
  {
    if (radius < 0 && length < 0)
      return;
    cout << "--------------------------------------" << endl;
    if (radius > 0)
    {
      cout << "Object Name: sph or ssph" << endl;
      cout << "Radius: " << radius << endl;
    }
    else
    {
      cout << "Object Name: box" << endl;
      cout << "Length: " << length << endl << "Width: " << width << endl << "Height: " << height << endl;
    }
    cout << "Coordinates x: " << x << " y: " << y << " z: " << z << endl;
  }

  /* set all information to the -1 which means this is not an object */
  void
  del ()
  {
    length = width = height = radius = x = y = z = -1;
  }
};

void
addObject (BufferedPort<Bottle>& obj, WorldObj *objects, int position)
{

  /* To avoid to create more objects than object count limit which is 20 */
  if (position > 20)
  {
    cout << "Object count limit is EXCEEDED!" << endl;
    return;
  }
  Bottle& bottle = obj.prepare ();
  double a, b, c, r;
  double x, y, z;
  int red, green, blue;
  char *name = (char *)malloc (sizeof(char) * 20);
  bottle.clear ();
  cout << "Please enter the name of the object that you want to add" << endl;

  /* To select the type of an object wanted to be created, and enter
   * the required information of an object
   */
  while (true)
  {
    cout << "Name (e.g: box, ssph, sph): ";
    cin >> name;
    if (!strcmp ("ssph", name) || !strcmp ("sph", name))
    {
      cout << "Please enter the radius of " << name << " object" << endl;
      cout << "r: ";
      cin >> r;
      break;
    }
    else if (!strcmp ("box", name))
    {
      cout << "Please enter the dimensions of " << name << " object" << endl;
      cout << "a: ";
      cin >> a;
      cout << "b: ";
      cin >> b;
      cout << "c: ";
      cin >> c;
      break;
    }
    else
      cout << "INVALID NAME!!\nPlease re-enter the name of the object" << endl;
  }
  cout << "Please enter the position of " << name << " object" << endl;
  cout << "x: ";
  cin >> x;
  cout << "y: ";
  cin >> y;
  cout << "z: ";
  cin >> z;
  cout << "Please enter the color bits of " << name << " object" << endl;
  cout << "Red: ";
  cin >> red;
  cout << "Green: ";
  cin >> green;
  cout << "Blue: ";
  cin >> blue;

  /* ordering the information to create an object and sending it to the
   * /icubSim/world port
   */
  bottle.addString ("world");
  bottle.addString ("mk");
  bottle.addString (name);
  if (!strcmp ("box", name))
  {
    bottle.addDouble (a);
    bottle.addDouble (b);
    bottle.addDouble (c);
    objects[position].addLWH (a, b, c);
  }
  else
  {
    bottle.addDouble (r);
    objects[position].addRadius (r);
  }
  bottle.addDouble (x);
  bottle.addDouble (y);
  bottle.addDouble (z);

  objects[position].addPositionCoordinates (x, y, z);

  bottle.addInt (red);
  bottle.addInt (green);
  bottle.addInt (blue);

  obj.write ();
}

/* Selection of an operation wanted to be achieved */
int
selectOperation ()
{
  int op;
  cout << endl;
  cout << "##########################################################" << endl;
  cout << "Please select the operation from below list:" << endl;
  cout << "Operation #0: Exit from program" << endl;
  cout << "Operation #1: Hide the object" << endl;
  cout << "Operation #2: Push the object from table" << endl;
  cout << "Operation #3: Look an object from different angles" << endl;
  cout << "Operation #4: Add an object to the iCub Simulator world" << endl;
  cout << "Operation #5: Print the features of all objects in iCub Simulator world" << endl;
  cout << "Operation #6: Delete all objects from iCub Simulator world" << endl;
  cout << "Operation #7: Go all joints of iCub to the beginning position" << endl;
  cout << "Operation #8: Pick up an object" << endl;
  cout << "Your selection(e.g: 1): ";
  cin >> op;
  return op;
}

/* deletes all object in iCub simulator world by sending "world del all"
 * command to the /icubSim/world port
 */
void
deleteObjects (BufferedPort<Bottle>& obj)
{
  Bottle& bottle = obj.prepare ();
  bottle.clear ();

  bottle.addString ("world");
  bottle.addString ("del");
  bottle.addString ("all");

  obj.write ();
}

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

void
hideSwap (WorldObj* objects, BufferedPort<Vector>& position, int HS)
{
  int order;

  cout << "Specify the object number in order of creation to hide or swap" << endl;
  cin >> order;

  if (objects[order - 1].getRadius () < 0 && objects[order - 1].getLength () < 0)
    return;

  Vector& posVec = position.prepare ();
  posVec.resize (5);

  if (objects[order - 1].getRadius () < 0)
  {
    posVec[0] = objects[order - 1].getWidth () / 2;
  }
  else
  {
    posVec[0] = objects[order - 1].getRadius ();
  }

  posVec[1] = objects[order - 1].getX ();
  posVec[2] = objects[order - 1].getY ();
  posVec[3] = objects[order - 1].getZ ();
  posVec[4] = HS;

  position.write ();
}

void
goToHomePosition (BufferedPort<Vector>& position, int H)
{
  Vector& posVec = position.prepare ();
  posVec.resize (5);

  posVec[4] = H;

  position.write ();
}

void
pickUpObject (WorldObj* objects, BufferedPort<Vector>& position, int H)
{
  int order;

  cout << "Specify the object number in order of creation to pick up" << endl;
  cin >> order;

  if (objects[order - 1].getRadius () < 0 && objects[order - 1].getLength () < 0)
    return;

  Vector& posVec = position.prepare ();
  posVec.resize (5);

  if (objects[order - 1].getRadius () < 0)
  {
    posVec[0] = objects[order - 1].getWidth () / 2;
  }
  else
  {
    posVec[0] = objects[order - 1].getRadius ();
  }

  posVec[1] = objects[order - 1].getX ();
  posVec[2] = objects[order - 1].getY ();
  posVec[3] = objects[order - 1].getZ ();
  posVec[4] = H;

  position.write ();
}

int
main ()
{
  Network yarp; // yarp connection

  /* Port for getting the information about focusing is achieved or not,
   * from finished port from search.cpp
   */
  BufferedPort<Vector> control;
  BufferedPort<Bottle> obj;
  BufferedPort<Vector> sendPosition;
  WorldObj objects[20]; // for object information
  int objectCount = 0; // current object count
  control.open ("/target/control"); // opening port
  obj.open ("/target/world"); // opening port
  sendPosition.open ("/target/pos");
  Network::connect ("/target/finished", "/target/control"); // connection
  Network::connect ("/target/world", "/icubSim/world"); // connection

  // for torso motions
  Property torso;
  torso.put ("device", "remote_controlboard");
  torso.put ("local", "/client/torso");
  torso.put ("remote", "/icubSim/torso");

  PolyDriver robotTorso (torso);

  // controls whether valid or not
  if (!robotTorso.isValid ())
  {
    cout << "Connection cannot be established with head!" << endl;
    return (-1);
  }

  IPositionControl *pos;
  IEncoders *ienc;

  bool ok = robotTorso.view (pos) && robotTorso.view (ienc);

  if (!ok)
  {
    cout << "Cannot get interface to robot torso!" << endl;
    robotTorso.close ();
    return (-1);
  }

  int joints;
  pos->getAxes (&joints);
  Vector acc;
  Vector speed;
  Vector command;
  command.resize (joints);
  acc.resize (joints);
  speed.resize (joints);

  int i;

  // setting the speeds, accelerations and home position of torso
  for (i = 0; i < joints; i++)
  {
    speed[i] = 10;
    command[i] = 0;
    acc[i] = 50;
  }
  pos->setRefSpeeds (speed.data ());
  pos->setRefAccelerations (acc.data ());
  pos->positionMove (command.data ());
  waitUntilFinish (pos);

  int op = 0;

  // operations according to the operation number which is selected by
  // user
  while (true)
  {
    op = selectOperation ();

    if (op == 8)
    {
      pickUpObject (objects, sendPosition, 3);
    }

    else if (op == 7)
    {
      goToHomePosition (sendPosition, 2);
    }

    /* deletes all objects from iCub simulator world, sets the
     * objectCount to zero, and deletes all object information from
     * WorldObj class
     */

    else if (op == 6)
    {
      objectCount = 0;
      deleteObjects (obj);
      for (int i = 0; i < 20; i++)
      {
        objects[i].del ();
      }
    }

    /* prints all information of the current objects in iCub
     * simulator world
     */
    else if (op == 5)
    {
      for (int i = 0; i < 20; i++)
      {
        objects[i].printObjects ();
      }
    }

    /* goes to addObject function to create an object in iCub
     * simulator world
     */
    else if (op == 4)
    {
      addObject (obj, objects, objectCount++);
    }

    /* changes the torso position randomly after getting the
     * focusing completed signal from search.cpp to make iCub focus
     * to the object at most ten times
     */
    else if (op == 3)
    {
      int count = 0;
      while (count < 3)
      {
        Vector *done = control.read ();
        if ((*done)[0] == 1)
        {
          command[0] = rand () % 50 - 25;
          command[1] = rand () % 60 - 30;
          command[2] = rand () % 55 - 5;
          pos->positionMove (command.data ());
          waitUntilFinish (pos);
          count++;
        }
      }
      cout << "Operation #3 is FINISHED!" << endl;
    }

    else if (op == 2)
    {
      hideSwap (objects, sendPosition, 0);
    }

    else if (op == 1)
    {
      hideSwap (objects, sendPosition, 1);
    }

    // closes the PolyDriver object and exits
    else if (op == 0)
    {
      objectCount = 0;
      deleteObjects (obj);
      for (int i = 0; i < 20; i++)
      {
        objects[i].del ();
      }
      cout << "Bye Bye!!!" << endl;
      robotTorso.close ();
      break;
    }

    // prints that the selection is invalid
    else
    {
      cout << "-------INVALID OPERATION SELECTION-------" << endl;
    }
  }

  return 0;
}
