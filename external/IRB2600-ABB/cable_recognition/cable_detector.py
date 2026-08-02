#!/usr/bin/env python

import sys
import copy
import pyrealsense2 as rs
import rospy
import moveit_commander
from moveit_msgs.msg import MoveGroupActionResult
from moveit_msgs.msg import DisplayTrajectory
import geometry_msgs.msg 
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
import std_msgs.msg
from moveit_commander.conversions import pose_to_list, list_to_pose
import pandas as pd
import camera as cm


class PickNPlaceTutorial():
    """PickNPlaceTutorial"""
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        #self.sub = rospy.Subscriber("/connector/position_connector",geometry_msgs.msg.Point, callback=self.callback_pose)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot_group = moveit_commander.MoveGroupCommander('indy7')
        self.hand_group = moveit_commander.MoveGroupCommander('hand')

        self.plan_result_pub = rospy.Publisher("/move_group/result", MoveGroupActionResult, queue_size=1)
        display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",DisplayTrajectory,queue_size=20)
        # Misc variables
        self.box_name = ''

    #def callback_pose(msg:geometry_msgs.msg.Point):
            #global x
            #global y
            #x,y = msg.x,msg.y

    def hold_hand(self, target_name):
        touch_links = ['left_inner_finger', 'right_inner_finger']
        self.hand_group.attach_object(target_name, 'robotiq_85_base_link', touch_links=touch_links)

    def release_hand(self, target_name):
        self.hand_group.detach_object(target_name)

    def jmove_to_pose_goal(self, goal):
        pose_goal = goal
        self.robot_group.set_pose_target(pose_goal)
        success = self.robot_group.go(wait=True)
        self.robot_group.stop()
        self.robot_group.clear_pose_targets()

    def jmove_to_joint_goal(self, goal):
        joint_goal = self.robot_group.get_current_joint_values()
        joint_goal[0] = goal[0]
        joint_goal[1] = goal[1]
        joint_goal[2] = goal[2]
        joint_goal[3] = goal[3]
        joint_goal[4] = goal[4]
        joint_goal[5] = goal[5]
        self.robot_group.go(joint_goal, wait=True)
        
    def tmove_to_pose_goal(self, pose_goal):
        waypoints = []
        waypoints.append(self.robot_group.get_current_pose().pose)
        waypoints.append(pose_goal)

        plan, _ = self.robot_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

        msg = MoveGroupActionResult()
        msg.result.planned_trajectory = plan
        self.plan_result_pub.publish(msg)
        self.robot_group.execute(plan)
    
    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_box(self, name, pose_stamp, size=(0.05, 0.05, 0.05)):
        self.scene.add_box(name, pose_stamp, size=size)
        self.wait_for_state_update(box_name=name, box_is_known=True)


def main():
    try:
        def read_points():
            depth_img, color_img = cm.take_picture()
            df = cm.get_points(color_img)
            #df = pd.read_csv("puntos.csv")
            pos = df[["X","Y"]].to_numpy().tolist()
            return pos
        pnp = PickNPlaceTutorial()
        #raw_input()
        rospy.init_node('pick_n_place_tutorial', anonymous=True)
        
        

#Poses readed
        pnp.jmove_to_joint_goal([0, 0, -pi/2, 0, -pi/2, 0])
        pos = read_points()
        for i in range(len(pos)):
            act = pos[i]
            cube_0 = geometry_msgs.msg.PoseStamped()
            cube_0.header.frame_id = pnp.robot_group.get_planning_frame()
            cube_0.pose.position.x =act[0]
            cube_0.pose.position.y =act[1]
            cube_0.pose.position.z =0
            cube_0.pose.orientation.w = 1.0                            

            pnp.add_box(name='cube_0', pose_stamp=cube_0)
            rospy.sleep(0.5)
            pnp.tmove_to_pose_goal(list_to_pose([act[0], act[1], 0.18, 0, -pi, -pi/2]))
            print(list_to_pose([act[0], act[1], 0.18, 0, -pi, -pi/2]))
        pnp.jmove_to_joint_goal([0, 0, -pi/2, 0, -pi/2, 0])
#Planning to a Joint Goal
        #pnp.jmove_to_joint_goal([0, 0, -pi/2, 0, -pi/2, 0])
        ##pnp.jmove_to_pose_goal(list_to_pose([0.35, -0.10, 0.18]))
        #pnp.tmove_to_pose_goal(list_to_pose([1, 1, 0.18, 0, -pi, -pi/2]))
        #pnp.hold_hand('cube_0')
        #pnp.tmove_to_pose_goal(list_to_pose([0.35, -0.20, 0.28, 0, -pi, -pi/2]))
        ##pnp.jmove_to_pose_goal([1.0, 0.35, 0.10, 0.23])

        ##pnp.jmove_to_joint_goal([0.65, -0.10, 0.48, 0, -pi, -pi/2])
        #pnp.tmove_to_pose_goal(list_to_pose([0.65, -0.10, 0.18, 0, -pi, -pi/2]))
        #pnp.release_hand('cube_0')
        #pnp.tmove_to_pose_goal(list_to_pose([0.65, -0.10, 0.28, 0, -pi, -pi/2]))

        #pnp.jmove_to_joint_goal([0, 0, -pi/2, 0, -pi/2, 0])

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    else:
        print("============ Python pick place demo complete!")
if __name__ == '__main__':
  main()
