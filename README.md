# piro_simulation
scripts for simulate piro on gazebo

Check with rospack list in terminal to see if the following packages:

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

Step by step guide to run the package:
  -Install ROS kinetic
  -Create a ros workspace
  -Inside the workspace create a folder for your packages (src)
  -Inside your src folder run git init to declare as a github workspace
  -Git clone the url of the project
  -Install some packages that are necessary in order to run the project:
    -sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

    -sudo apt-get install ros-kinetic-koint-trajectory-controller

    -sudo apt-get install ros-kinetic-joint-state-controller

    -sudo apt-get install ros-kinetic-effort-controller

    -sudo apt-get install ros-kinetic-position-controller

    -sudo apt-get install ros-kinetic-velocity-controller
  -In order to use joystick, follow the instructions below:
    -Find the joystick-driver
    -Go to the github page
    -Clone the repository into the src folder and do the following:
      cd ~/catkin_ws/src
      git clone [URL]
      cd ~/catkin_ws
      sudo apt-get install libusb-dev
    -Remove spacenav_node and wiimote that are inside the joystick-driver folder, they presented bugs
    -Inside catkin_ws do a catkin_make
  -Launch the world
  The version of gazebo is the 7 and ros is kinetic    
