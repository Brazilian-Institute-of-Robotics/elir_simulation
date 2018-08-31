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

## Avaiable Applications

RVIZ Visualization:

```
$ roslaunch elir_description elir_view.launch
```

Gazebo Simulation:
```
$ roslaunch elir_gazebo elir_world.launch
```

## Joystick Application
  Find your joystick-driver and clone the repository into the src folder 

  ```
  $ cd ~/catkin_ws/src
  ```

  ```
  $ git clone [URL]
  ```

  ```
  $ cd ~/catkin_ws
  ```

  And install libusb-dev
  ```
  $ sudo apt-get install libusb-dev
  ```
  
  1. Remove spacenav_node and wiimote that are inside the joystick-driver folder, they presented bugs
    
  2. Inside catkin_ws do a catkin_make
  
  3. Launch the world
    

## Contributions
* **Cleber Couto** - [clebercoutof](https://github.com/clebercoutof)
* **Daniel Pinha** - [DanielPínha](https://github.com/DanielPinha)

## Credits
* **Marco Reis** - [mhar-vell](https://github.com/mhar-vell)
* **Pedro Xavier** - [pxalcantara](https://github.com/pxalcantara)
