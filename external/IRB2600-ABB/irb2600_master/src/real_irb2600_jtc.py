#!/usr/bin/env python3

import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import sys

def perform_trajectory():

    rospy.init_node('abb_trajectory_publisher') #Node definition

    contoller_name='/joint_path_command'

    trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10) #Publisher Definition

    argv = sys.argv[1:] #It will read the webshell 

    abb_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'] #Name of joints

    goal_positions = [ float(argv[0]) , float(argv[1]) , float(argv[2]) ,float(argv[3] ) ,

                        float(argv[4]) , float(argv[5])  ] #Positions in radians where to move

 

    rospy.loginfo("Goal Position set lets go ! ")

    rospy.sleep(1)

    trajectory_msg = JointTrajectory() #JointTrajectory object declaration

    trajectory_msg.joint_names = abb_joints

    trajectory_msg.points.append(JointTrajectoryPoint())

    trajectory_msg.points[0].positions = goal_positions

    trajectory_msg.points[0].velocities = [0.0 for i in abb_joints]

    trajectory_msg.points[0].accelerations = [0.0 for i in abb_joints]

    trajectory_msg.points[0].time_from_start = rospy.Duration(3)

    rospy.sleep(1)

    trajectory_publihser.publish(trajectory_msg)


if __name__ == '__main__':

    perform_trajectory()