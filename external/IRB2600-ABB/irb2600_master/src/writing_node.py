#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from   sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Point
import os
import math
import moveit_commander
from visualization_msgs.msg import Marker, MarkerArray

global pos
pos =       [0.0,0.0,0.0,0.0,0.0,0.0]

global fig
fig = '_none'

global pen
pen = 1.03 - 0.008

global marker_array
marker_array = MarkerArray()

global marker
marker = Marker()
marker.header.frame_id = "base_link"
marker.type = marker.POINTS
marker.action = marker.ADD
marker.scale.x = 0.0045
marker.scale.y = 0.0045
marker.scale.z = 0
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 0.0
marker.color.a = 1.0
marker.lifetime = rospy.Duration(10)

#Recibimos del Subscriber un msg de tipo JointState de moveit y posteriormente lo publicamos con el Publisher como goal
def state_position(goal_state: JointState):
    global pos
    if fig != '_none': 
        plan_marker()
    else:
        marker.points.clear()
        marker_array.markers.append(marker)

global marker_id
marker_id = 0

def plan_marker():
    global marker_array
    global marker
    global pen
    global marker_id
    
    pose = group.get_current_pose(group.get_end_effector_link())

    if (pose.pose.position.z <= pen + 0.001) and (pose.pose.position.z >= pen - 0.001):
        p = Point() 
        p = pose.pose.position
        p.z = 0.25
        
        marker.points.append(p)
        marker.id = marker_id
        marker_array.markers.append(marker)

        # Publicamos solo el último punto para mantener el rendimiento
        marker_array.markers = [marker]
        marker_pub.publish(marker_array)

        rospy.sleep(0.001)  # Ajusta este valor según la velocidad deseada

        # Incrementamos el ID para el próximo marcador
        marker_id += 1


def figure(data_figure : str):
    global fig
    global pen
    if "," in str(data_figure.data):
        fig = (str(data_figure.data)).split(",")[0]
        pen = float((str(data_figure.data)).split(",")[1])
    else:
        fig = str(data_figure.data)

if __name__ == "__main__":
    rospy.sleep(2)
    rospy.init_node("writing_node")
    moveit_commander.roscpp_initialize(sys.argv)
    marker_pub      = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 10000)
    subGoalState    = rospy.Subscriber("/joint_states", JointState, callback = state_position)
    subWritingData  = rospy.Subscriber("/figure_writing", String, callback = figure)
    group           = moveit_commander.MoveGroupCommander("irb2600_arm")

    rospy.logwarn("The writing_node has been started")
    rospy.spin()