# ELIR Simulation

Simulate ELIR on gazebo!

## Requirements

  * [Ubuntu 16.04 LTS (Xenial)](http://releases.ubuntu.com/14.04/) 
  * [ROS Kinetic Kame](http://wiki.ros.org/indigo/Installation/Ubuntu) 
  * [ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
  * [Gazebo 7+](http://gazebosim.org/download)



## Installation
Go to the install folder
```
$ cd ~/catkin_ws/src/elir_simulation/install
```

Run the insall.sh file 
```
$ ./install.sh
```
Remember to compile you do a catkin_make in your workspace.

Go to the catkin_ws:
```
$ cd ~/catkin_ws
```
Run:
```
$ catkin_make
```
Case ROS doesn't find the package, remember to source your setup.bash file
```
$ source ~/catkin_ws/devel/setup.bash
```

### Installed Packages
You can check with rospack list comand to see if the packages were installed:

* Velocity_controllers

* position_controllers

* gazebo_ros

* gazebo_ros_control

* pluginlib

* librviz_tutorial

* Xacro

* ps3joy

* joint_state_controller

* joint_state_publisher


## Included Packages and features
* elir_control - Contains the configurations for the elir joint controllers
* elir_description - Contains the URDF file for Elir Robot, the meshes, and one script that convert this model to collada, in order to view it in OpenRave
* elir_gazebo - Used to simulation with gazebo
* simulation_control - Control applications for simulation 

## Avaiable Applications

RVIZ Visualization:

```
$ roslaunch elir_description elir_view.launch
```

Gazebo Simulation:
```
$ roslaunch elir_gazebo elir_world.launch
```

MoveIt! Interaction with gazebo:
```
$ roslaunch moveit_with_gazebo new_moveit_planning_execution.launch
```

### Control Applications
The simulation_control package is responsible for moving the robot on the line through keyboard user input, in order to move the robot on the line use :

```
$ roslaunch simulation__control simulation_line_control.launch
```

To do the robot stretch, in order to push up the central unit on the line,, based in the keyboard input, run


```
$ roslaunch simulation_control robot_extend_simulate.launch
```

In order to enable the service to open and close the claw

```
$ rosrun simulation_control open_close_claw_srv.py
```

## Contributions
* **Cleber Couto** - [clebercoutof](https://github.com/clebercoutof)
* **Daniel Pinha** - [DanielPínha](https://github.com/DanielPinha)

## Credits
* **Marco Reis** - [mhar-vell](https://github.com/mhar-vell)
* **Pedro Xavier** - [pxalcantara](https://github.com/pxalcantara)
