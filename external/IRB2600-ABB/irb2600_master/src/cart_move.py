#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import copy


def perform_trajectory():

    argv = sys.argv[1:] #It will read the webshell 

    wpose = group.get_current_pose().pose
    waypoints = []
    # We get the joint values from the group and change some of the values:
    goal_pose = [ float(argv[0]) , float(argv[1]) , float(argv[2]) ]
    
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    wpose.position.x = goal_pose[0]
    wpose.position.y = goal_pose[1]
    wpose.position.z = goal_pose[2]
    waypoints.append(copy.deepcopy(wpose))

    plan  = group.compute_cartesian_path(waypoints, 0.1, 0.0)[0]

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    group.execute(plan, wait=True)
    rospy.loginfo("Planning succesfully executed.\n")
    rospy.logwarn(wpose)
    rospy.sleep(1)


if __name__ == '__main__':

    rospy.init_node('abb_trajectory_publisher') #Node definition

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()    
    group = moveit_commander.MoveGroupCommander("irb2600_arm")
    perform_trajectory()