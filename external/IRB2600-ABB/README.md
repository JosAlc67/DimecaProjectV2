# IRB2600-ABB-ROS1
<h1 style="border:none"> RISE ABB IRB-2600 ROS Manipulation Package - Industrial robot for ESPOL</h1>
&copy; 2022, Francisco Yumbla - Javier Pagalo

<hr>

## 1. How to Install

### 1.1. System Requirements

This package is written an tested on **Ubuntu 20.04 + ROS Noetic** environment. Dependencies are also for this environment.

### 1.2. Dependencies Prerequisites

There are a number of dependencies in this package, since the ABB robot is operated by ROS-Industrial package. Please install all the packages listed below in your Ubuntu PC, in the given order. These packages can be installed by `apt` package manager.

* ros-noetic-desktop-full
* ros-noetic-industrial-core
* ros-noetic-industrial-msgs
* ros-noetic-industrial-robot-client
* ros-noetic-industrial-robot-simulator
* ros-noetic-industrial-utils
* ros-noetic-moveit
* ros-noetic-joint-state-publisheser-gui
* ros-noetic-joint-trajectory-controller

The following packages are also neccesary but you must install it by cloning from the github repository and building your workspace.

* ros-melodic-abb
* ros-noetic-abb-driver

We also need to install the following projects to make the trajectories.
```
sudo pip install spatialmath-rospy
```
```
sudo pip install spatialmath-python
```
```
sudo pip install -U numpy
```
These projects belong respectively to [SpatialMath Rospy](https://pypi.org/project/spatialmath-rospy/) and [SpatialMath Python](https://pypi.org/project/spatialmath-python/).

Now, Extract the metapackage `IRB2600-ABB` into `${ros_workspace}/src`. `catkin_make` your workspace.

We also need to add the [ABB](https://github.com/ros-industrial/abb.git) repository from 
ROS-Industrial into `${ros_workspace}/src`.

**WARNING: If you planing use grippers with this robot. You need copy the [gripper package](https://github.com/fryumbla/Robotiq-grippers.git) (In construction)**

*If you want control with the joystick control. You need install: `sudo pip install ds4drv`


## 2. Structure of Packages

* **irb2600_description:** This package contains the URDF and XACRO files for diferents configuration of the robot with grippers.
* **irb2600_master:** This pasckage contains a diferrents examples of motion used MoveIt and the joystick&keyboard control of the real robot.
* **irb2600_vrep (In construction):** This package contains the communication with V-REP simulator including examples and scenes
* **irb2600_configuration_moveit:** This package contains the diferent MoveIt configuration of diferents configuration of the robot

## 3. How to Use

### 3.1. Simulation

Open terminal and `roscore` and `Enter`. 

#### 3.1.1 Rviz Visualization

1. Launch the robot visualization Rviz
   ```
   roslaunch irb2600_description irb2600_display.launch
   ```

2. Launch the robot with Moveit configuration
   ```
   roslaunch irb2600_moveit demo.launch
   ```


#### 3.1 Gazebo Simulation (in construction)

1. Launch the robot in gazebo
   ```
   roslaunch irb2600_gazebo irb2600_gazebo.launch
   ```

### 3.2. Real Robot

The real robot work with the moveit_configuration package for precaution collision in our workspace or environment (In construction)

Setup the Robot and turn on. 

1. Launch the robot
   ```
   roslaunch irb2600_description real_robot.launch
   ```

2. If you requires to launch the Gazebo empty world
   ```
   roslaunch irb2600_gazebo irb2600_gazebo.launch
   ```
3. To command the robot from the command prompt, send the joint configuration

   ```
   rosrun irb2600_master real_irb2600_jtc.py 0 0 0 0 0 0 
   ```


este para correr sin gazebo

   roslaunch irb2600_description real_robot.launch robot:=true



