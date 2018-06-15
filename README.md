# ELIR Simulation

Scripts for simulate ELIR on gazebo!

## Installing Package Dependencies
Considering that you already have ROS Kinetic installed, go to install folder:
```
$ ~/catkin_ws/src/elir_simulation/install
```

Run the insall.sh file

```
$ ./install.sh
```

You can check with rospack list in terminal to see if the following packages were installed:
```
Velocity_controllers

position_controllers

gazebo_ros

gazebo_ros_control

pluginlib

librviz_tutorial

Xacro

ps3joy

joint_state_controller

joint_state_publisher
```
## Avaiable Applications

In order to visualize your robot on rviz:

```
$ roslaunch mybot_description piro_view.launch
```

In order to run your World on gazebo:

```
$ mybot_gazebo mybot_world.launch
```

## Manually installing the Dependencies
Step by step guide to install package dependencies:

1. Install ROS kinetic

2. Create a ros workspace

3. Inside the workspace create a folder for your packages (src)

4. Inside your src folder run git init to declare as a github workspace

5. Git clone the url of the project

6. Install some packages that are necessary in order to run the project:
  
  ```
    $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
  ```

  ```
    $ sudo apt-get install ros-kinetic-koint-trajectory-controller
  ```

  ```
    $ sudo apt-get install ros-kinetic-joint-state-controller
  ```

  ```
    $ sudo apt-get install ros-kinetic-effort-controller
  ```

  ```
    $ sudo apt-get install ros-kinetic-position-controller
  ```

  ```
    $ sudo apt-get install ros-kinetic-velocity-controller
  ```

## Joystick

  In order to use joystick, follow the instructions below:
  Find the joystick-driver
  Go to the github page
  Clone the repository into the src folder and do the following:
  ```
  $ cd ~/catkin_ws/src
  ```

  ```
  $ git clone [URL]
  ```

  ```
  $ cd ~/catkin_ws
  ```

  ```
  $ sudo apt-get install libusb-dev
  ```
  
  1. Remove spacenav_node and wiimote that are inside the joystick-driver folder, they presented bugs
    
  2. Inside catkin_ws do a catkin_make
  
  3. Launch the world

  ## Versions
  The version of gazebo is the 7 and [ROS] is kinetic    
