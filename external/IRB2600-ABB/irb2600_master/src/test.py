#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import math

# Altura del lapiz
global pen 
pen = 1.0 

global quit
quit = 0

global theta
theta = 0

global t
t = 0.1

#Altura máxima a la que llegará cada letra en Y
global y_h
y_h = 1.2

#Tamaño de cada letra en ancho y alto
global size
size = 0.1

#Espacio entre cada letra
global space
space = 0.02

#Altura cuando se levanta el l
def home():
    # We get the joint values from the group and change some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 1.57
    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)
    rospy.loginfo("The robotic arm is at home position.")

def loginfog(msg: str):
    rospy.loginfo("\033[92m%s\033[0m" % msg)

def print_plan(w: list, s: str):
    message = "\n--------------------------------------------------------------"
    for i in range(len(w)):
        message += """
Pose {0}:\n{1}
--------------------------------------------------------------""".format( i , w[i])
    
    rospy.loginfo(message)
    loginfog("Drawing a " + s)

def pen_up_down(wpose, waypoints : list):
    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints

def up_pen(wpose, waypoints : list):
    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints

def down_pen(wpose, waypoints : list):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints
    
def move_pen(wpose, waypoints : list, d_x : float, d_y: float, d_z : float = 0):
    #Se copia la pose actual para únicamente modificar las coordenadas cartesianas y que la orientación
    #del efector final no se vea modificada, de esta manera mantenemos el lápiz perpendicular al suelo

    wpose.position.y -= d_x
    wpose.position.x = (y_h if d_y == y_h else
                       (wpose.position.x + d_y))
    if (d_z != 0):
        wpose.position.z = d_z

    waypoints.append(copy.deepcopy(wpose))

    return (wpose, waypoints)

def set_pen(wpose, waypoints : list, p_x : float, p_y: float, p_z : float = 0):
    
    wpose.position.y = -p_x
    wpose.position.x = p_y
    wpose.position.z = p_z
    waypoints.append(copy.deepcopy(wpose))

    return (wpose, waypoints)




#By executing this file we can make the robot move to several preconfigured positions in Cartesian coordinates, in the order in which they are in the file
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planing_node', anonymous=True)
rate = rospy.Rate(10)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("irb2600_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
data_writing_publisher = rospy.Publisher('/figure_writing', String, queue_size=2)
data_writing_publisher.publish(("_none," + str(pen)))
# Calling ``stop()`` ensures that there is no residual movement
group.stop()
wpose = group.get_current_pose().pose
waypoints = []
(wpose, waypoints) = set_pen(wpose, waypoints, 0, y_h, pen)

data_writing_publisher.publish("_" + str("test").lower() + "," + str(pen))
rospy.sleep(1)

plan  = group.compute_cartesian_path(waypoints, t, 0.0)[0]

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)

group.execute(plan, wait=True)
rospy.loginfo("Planning succesfully executed.\n")
rospy.sleep(1)
data_writing_publisher.publish("_none")

