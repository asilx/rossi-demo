#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_keyboard_teleoperator')
import rospy, sys, select, termios, tty, os

import signal
import time

from trajectory_msgs.msg import *
from pr2_mechanism_msgs.msg import *
from pr2_mechanism_msgs.srv import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import Twist


usage = """
Reading from the keyboard
--------------------------

Moving the Base:

   7    8    9
   4    5    6
   1    2    3

Moving the Arm's Joints:

   Shoulder pan joint: e/d
   Shoulder lift joint: r/f
   Upper arm roll joint: t/g
   Elbow flex joint: y/h
   Forearm roll joint: u/j
   Wrist flex joint: i/k
   Wrist roll joint: o/l

Moving The Head:

   Panning: v/b
   Tilting: n/m

Moving The Torso:

   Lower/Raise: ,/.

Using the gripper:

   Open/Close (current arm): c

Parameters:

   Switching between arms: z/x
   Increasing/Decreasing delta (in radians): q/a

CTRL-C to quit
(p) to reprint USAGE
--------------------------
"""

#initialize the arm, gripper, and head controllers
pub_right = rospy.Publisher('r_arm_controller/command', JointTrajectory, latch=True)
pub_left = rospy.Publisher('l_arm_controller/command', JointTrajectory, latch=True)
rhand_pub = rospy.Publisher('r_gripper_controller/command', Pr2GripperCommand)
lhand_pub = rospy.Publisher('l_gripper_controller/command', Pr2GripperCommand)
head_pub = rospy.Publisher('head_traj_controller/command', JointTrajectory, latch=True)
torso_pub = rospy.Publisher('torso_controller/command', JointTrajectory)

#initialize the open gripper command
open_cmd = Pr2GripperCommand()
open_cmd.position = 0.08     #open position
open_cmd.max_effort = -1.0   #no limit on force

#initialize the close gripper command
close_cmd = Pr2GripperCommand()
close_cmd.position = -100.00   #closed position
close_cmd.max_effort = -1.0    #no limit on force


jointBindings = {
		'e':(1,0),
		'd':(2,0),
		'r':(3,0),
		'f':(4,0),
		't':(5,0),
		'g':(6,0),
		'y':(7,0),
		'h':(8,0),
		'u':(9,0),
		'j':(10,0),
		'i':(11,0),
		'k':(12,0),
		'o':(13,0),
		'l':(14,0),
	       }

moveBindings = {
		'8':(1,0),
		'9':(1,-1),
		'4':(0,1),
		'6':(0,-1),
		'7':(1,1),
		'2':(-1,0),
		'3':(-1,1),
		'1':(-1,-1),
	       }

paramterBindings ={
		'z':(1,0),
		'x':(2,0),
		'q':(3,0),
		'a':(4,0),
		'p':(5,0),
	      }

headGripperBindings ={
		'c':(1,0),
		'v':(2,0),
		'b':(3,0),
		'n':(4,0),
                'm':(5,0),
		',':(6,0),
		'.':(7,0),
	      }

def handleArms(side, positions):
  traj = JointTrajectory()
  traj.joint_names = ["%s_shoulder_pan_joint" % side,
                      "%s_shoulder_lift_joint" % side,
                      "%s_upper_arm_roll_joint" % side,
                      "%s_elbow_flex_joint" % side,
                      "%s_forearm_roll_joint" % side,
                      "%s_wrist_flex_joint" % side,
                      "%s_wrist_roll_joint" % side]
  traj.points = []
  for p in positions:
    traj.points.append(JointTrajectoryPoint(positions = p[1:],
                                            velocities = [0.0] * (len(p) - 1),
                                            accelerations = [],
                                            time_from_start = rospy.Duration(p[0])))

  traj.header.stamp = rospy.get_rostime() + rospy.Duration(0.001)
  if(side == 'l'):
    pub_left.publish(traj)
  else:
    pub_right.publish(traj)


def handleTorso(x):
  p = [x]
  traj = JointTrajectory()
  traj.joint_names = ["torso_lift_joint"]
  traj.points = []
  traj.points.append(JointTrajectoryPoint(positions = p,
                                            velocities = [0.0] * (len(p)),
                                            accelerations = [],
                                            time_from_start = rospy.Duration(0.0)))

  traj.header.stamp = rospy.get_rostime() + rospy.Duration(0.001)
  torso_pub.publish(traj)


def handleHead(x,y):
  p = [x,y]
  traj = JointTrajectory()
  traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
  traj.points = []
  traj.points.append(JointTrajectoryPoint(positions = p,
                                            velocities = [0.0] * (len(p)),
                                            accelerations = [],
                                            time_from_start = rospy.Duration(0.0)))

  traj.header.stamp = rospy.get_rostime() + rospy.Duration(0.001)
  head_pub.publish(traj)


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def initialize():
	print 'Placing arms, grippers, torso, and head in default positions'
	initial_positions = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]]
	positions = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]]
	handleArms ('r', initial_positions)
        handleArms ('l', initial_positions)
	handleHead (0,0)
	handleTorso(0)

speed = .5
turn = 1


if __name__=="__main__":

	rospy.init_node('pr2_keyboard_teleoperator', anonymous = False)
	rospy.wait_for_service('pr2_controller_manager/switch_controller')
	  
    	settings = termios.tcgetattr(sys.stdin)
	pub_base = rospy.Publisher('/base_controller/command', Twist)

	delta = 0.1
	arm = 'r'
	x = 0
	th = 0

	l_joint_1 = 0
	l_joint_2 = 0
	l_joint_3 = 0
	l_joint_4 = 0
	l_joint_5 = 0
	l_joint_6 = 0
	l_joint_7 = 0
	r_joint_1 = 0
	r_joint_2 = 0
	r_joint_3 = 0
	r_joint_4 = 0
	r_joint_5 = 0
	r_joint_6 = 0
	r_joint_7 = 0

	r_gripper = 0 # 0 = closed, 1 = open
        l_gripper = 0 # 0 = closed, 1 = open

	head_pan = 0
	head_tilt = 0
	torso = 0

	initialize()

	try:
		print usage
		while(1):
			key = getKey()

			if key in paramterBindings.keys():
                          x = paramterBindings[key][0]
                          
                          if x == 1:
                            arm = 'l'
                            print 'Now controlling Left Arm'
			
                          elif x == 2:
                            arm = 'r'
                            print 'Now controlling Right Arm'

                          elif x == 3:
                            delta += 0.01;
                            print 'New delta value = ' + str(delta)
			
                          elif x == 4 and delta > 0.02:
                            delta -= 0.01;
                            print 'New delta value = ' + str(delta)

                          elif x == 5:
                            print usage
                            
                            
                        elif key in moveBindings.keys():
                          x = moveBindings[key][0]
                          th = moveBindings[key][1]
                          
                          twist = Twist()
                          twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
                          twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
                          pub_base.publish(twist)


			elif key in headGripperBindings.keys():
                          z = headGripperBindings[key][0]

                          if z == 1:
                            if arm == 'l':
                              if l_gripper == 0:
                                l_gripper = 1
                                lhand_pub.publish(open_cmd)
                              elif l_gripper == 1:
                                l_gripper = 0
                                lhand_pub.publish(close_cmd)
                              elif arm == 'r':
                                if r_gripper == 0:
                                  r_gripper = 1
                                  rhand_pub.publish(open_cmd)
                                elif r_gripper == 1:
                                  r_gripper = 0
                                  rhand_pub.publish(close_cmd)
                                  
                          elif z == 2:
                            head_tilt += 0.1
                            handleHead (head_tilt, head_pan)
                          elif z == 3:
                            head_tilt -= 0.1
                            handleHead (head_tilt, head_pan)
                          elif z == 4:
                            head_pan -= 0.1
                            handleHead (head_tilt, head_pan)
                          elif z == 5:
                            head_pan += 0.1
                            handleHead (head_tilt, head_pan)
                            
                          elif z == 6 and torso > 0:
                            torso -= 0.01
                            handleTorso(torso)
                          elif z == 7 and torso < 0.3:
                            torso += 0.01
                            handleTorso(torso)


			elif key in jointBindings.keys():
                          y = jointBindings[key][0]

				############# Shoulder Pan ##############
                          if y == 1:
                            if arm == 'l':
                              l_joint_1 += delta
                            elif arm == 'r':
                              r_joint_1 += delta

                          elif y == 2:
                            if arm == 'l':
                              l_joint_1 -= delta
                            elif arm == 'r':
                              r_joint_1 -= delta
                              
				############# Shoulder Lift ############
				   
                          elif y == 4:
                            if arm == 'l':
                              l_joint_2 += delta
                            elif arm == 'r':
                              r_joint_2 += delta

                          elif y == 3:
                            if arm == 'l':
                              l_joint_2 -= delta
                            elif arm == 'r':
                              r_joint_2 -= delta

				############## Upper Arm Roll ##########
                              
                          elif y == 5:
                            if arm == 'l':
                              l_joint_3 += delta
                            elif arm == 'r':
                              r_joint_3 += delta

                          elif y == 6:
                            if arm == 'l':
                              l_joint_3 -= delta
                            elif arm == 'r':
                              r_joint_3 -= delta
                                    
				############ Elbow Flex ###############
				   
                          elif y == 8:
                            if arm == 'l':
                              l_joint_4 += delta
                            elif arm == 'r':
                              r_joint_4 += delta
                              
                          elif y == 7:
                            if arm == 'l':
                              l_joint_4 -= delta
                            elif arm == 'r':
                              r_joint_4 -= delta
                              
				############## Forearm Roll ###########
                              
                          elif y == 9:
                            if arm == 'l':
                              l_joint_5 += delta
                            elif arm == 'r':
                              r_joint_5 += delta
                              
                          elif y == 10:
                            if arm == 'l':
                              l_joint_5 -= delta
                            elif arm == 'r':
                              r_joint_5 -= delta

				############## Wrist Flex #################
				   
                          elif y == 12:
                            if arm == 'l':
                              l_joint_6 += delta
                            elif arm == 'r':
                              r_joint_6 += delta

                          elif y == 11:
                            if arm == 'l':
                              l_joint_6 -= delta
                            elif arm == 'r':
                              r_joint_6 -= delta
                              
				############## Wrist Roll #################
				   
                          elif y == 13:
                            if arm == 'l':
                              l_joint_7 += delta
                            elif arm == 'r':
						r_joint_7 += delta

                          elif y == 14:
                            if arm == 'l':
                              l_joint_7 -= delta
                            elif arm == 'r':
                              r_joint_7 -= delta
                                
				############## Execute Arm Command ###########
                        
                          if arm == 'l':
                            positions = [[0.0,float(l_joint_1),float(l_joint_2),float(l_joint_3),float(l_joint_4),float(l_joint_5),float(l_joint_6),float(l_joint_7)]]
                          elif arm == 'r':
                            positions = [[0.0,float(r_joint_1),float(r_joint_2),float(r_joint_3),float(r_joint_4),float(r_joint_5),float(r_joint_6),float(r_joint_7)]]

                          handleArms (arm, positions)


                        else:
                          if (key == '\x03'):
                            break
	except:
		print usage

	finally:
		initialize()

		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub_base.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


