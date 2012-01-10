/*
 * RobotTorso.h
 *
 *  Created on: Aug 18, 2011
 *      Author: kadir
 */

#ifndef ROBOTTORSO_H_
#define ROBOTTORSO_H_

#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

class RobotTorso
{
private:
  TorsoClient *torso_client_;

public:
  //Action client initialization
  RobotTorso ();

  ~RobotTorso ();

  //tell the torso to go up
  void
  up ();

  //tell the torso to go down
  void
  down ();
};

#endif /* ROBOTTORSO_H_ */
