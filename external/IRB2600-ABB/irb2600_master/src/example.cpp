#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream> 
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
  
using namespace std;

geometry_msgs::PoseArray pos_connectors;
geometry_msgs::Pose goalpos;
float step = 0.00005;
float anglestep = 1.0;

void home(moveit::planning_interface::MoveGroupInterface& move_group) {
    // Get the current joint values
    std::vector<double> joint_group_positions;
    joint_group_positions = move_group.getCurrentJointValues();

    // Set the desired joint values (home position)
    joint_group_positions[0] = 0.0;
    joint_group_positions[1] = 0.0;
    joint_group_positions[2] = 0.0;
    joint_group_positions[3] = 0.0;
    joint_group_positions[4] = 1.57;
    joint_group_positions[5] = 0.0;

    // Plan to the new joint space goal
    move_group.setJointValueTarget(joint_group_positions);

    // Execute the plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group.move();
        ROS_INFO("The robotic arm is at home position.");
    } else {
        ROS_WARN("Failed to plan to home position.");
    }
}

void posCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  pos_connectors.poses = msg->poses;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_motion");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(100);

  // ros::Subscriber vel_sub = node_handle.subscribe("/input/vel", 10, velCallback);
  ros::Subscriber pos_sub = node_handle.subscribe("/planning_poses", 10, posCallback);
  // ros::Subscriber step_sub = node_handle.subscribe("/input/step", 10, stepCallback);
  // ros::Subscriber anglestep_sub = node_handle.subscribe("/input/anglestep", 10, anglestepCallback);
  
  
  moveit::planning_interface::MoveGroupInterface move_group("irb2600_arm");
  move_group.setPlanningTime(1);//0.1

  ros::AsyncSpinner spinner(10);
  spinner.start();

  geometry_msgs::Pose current_pose;
  current_pose = move_group.getCurrentPose().pose;

  // std::vector<geometry_msgs::Pose> waypoints;
  // waypoints.push_back(current_pose);

  // current_pose.position.z -= 0.2;
  // waypoints.push_back(current_pose);  // down

  // pos_connectors.poses = waypoints;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  int key=0;
  std::cout << std::fixed;
  std::cout.precision(3);

  home(move_group);


  while(ros::ok())
  {
    // std::cout << "x: " << goalpos.position.x << "  y: " << goalpos.position.y << "  z: " << goalpos.position.z << "  w: " << goalpos.orientation.w << "  xa: " << goalpos.orientation.x << "  ya: " << goalpos.orientation.y << "  za: " << goalpos.orientation.z <<std::endl;
    if (!(pos_connectors.poses).empty())
    {
      // std::cout << "PoseArray: \n" << pos_connectors<<std::endl;

      moveit_msgs::RobotTrajectory trajectory;
      double fraction = move_group.computeCartesianPath(pos_connectors.poses, step, 0, trajectory);
      std::cout << fraction <<std::endl;
      my_plan.trajectory_ = trajectory;

      // Execute the plan
      bool success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_WARN_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

      pos_connectors.poses.clear();    

    loop_rate.sleep();
    home(move_group);
    // ros::spinOnce();
    }
  }

  return 0;
}


