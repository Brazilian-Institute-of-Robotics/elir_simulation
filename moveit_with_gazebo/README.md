## MoveIt with Gazebo
This is the sample package used to interact MoveIt with gazebo!
The files 

## Features
With the new_moveit_planning execution launch, you'll be able to visualize your robot in Rviz and use the interactive marker to set a goal position.
```
$ roslaunch moveit_with_gazebo new_moveit_planning_execution.launch
```
In order to enable the move_group node that implements the move_group/MoveRequest service, run this command after your planning execution:
```
$ roslaunch moveit_with_gazebo custom_move_group.launch
```

This service allows you to send an X and Z goal and a configuration to an specific move_group.

#Gazebo Configuration

It's necessary that your robot has the trajectory controllers defined spawned in the gazebo simulation.
Your joints interface transmission is generaly described in your robot.urdf.xacro file, a simple position controlled joint looks like this:

```
<transmission name="tran_joint_1b">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1_b">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    
    <actuator name="servo12">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

Then you will create your trajectory control configuration file, save it inside your control folder, you can use the poisiton controllers or the effort controller,one effort controller shoud look like this:
```
robot:
  f_arm_trajectory_controller:
    type: effort_controllers/JointTrajectoryController
    publish_rate: 20
    joints:
      - joint1_f
      - joint2_f

    gains: # Required because we're controlling an effort interface
      joint1_f: {p: 32,  d: 0, i: 0.1, i_clamp: 1}
      joint2_f: {p: 32,  d: 0, i: 0.1, i_clamp: 1}

    state_publish_rate:  20          # Override default
    action_monitor_rate: 20            # Override default
    stop_trajectory_duration: 0        # Override default
```

Then, when using the gazebo simulation, spawn the controller using the controller spawner node as the examples.

#MoveIt! Configuration

Create one controlers.yaml file and paste it on the config folder, this file points to move it wich one is the action server declared in gazebo, the example of one arm controller is:
```
controller_manager_ns: controller_manager
controller_list:
  - name: robot/f_arm_trajectory_controller/
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint1_f
      - joint2_f

```
Then, one launch file is necessary to load this configuration, generally it is already created inside of the move it generated package, under the name of your_robot_moveit_controller_manager.launch.xml.It starts the MoveItSimpleControllerManager and loads the joint trajectory controller interface defined inside controllers.yaml.

```
<launch>
	<!-- Set the param that trajectory_execution_manager needs to find the controller plugin -->
	<arg name="moveit_controller_manager" default="moveit_simple_controller_manager/MoveItSimpleControllerManager" />
	<param name="moveit_controller_manager" value="$(arg moveit_controller_manager)"/>

	<!-- load controller_list -->
	<arg name="use_controller_manager" default="true" />
	<param name="use_controller_manager" value="$(arg use_controller_manager)" />

	<!-- Load joint controller configurations from YAML file to parameter server -->
	<rosparam file="$(find moveit_with_gazebo)/config/controllers.yaml"/>
</launch>
```

Now, create one moveit_planning_execution.launch file, that loads the planning execution.

```
<launch>
 # The planning and execution components of MoveIt! configured to 
 # publish the current configuration of the robot (simulated or real)
 # and the current state of the world as seen by the planner
 <include file="$(find moveit_with_gazebo)/launch/move_group.launch">
  <arg name="publish_monitored_planning_scene" value="true" />
 </include>
 # The visualization component of MoveIt!
 <include file="$(find moveit_with_gazebo)/launch/moveit_rviz.launch"/>
</launch>
```

#Running the tests
Launch your robot world, with the controllers and models. Then launch the moveit_planning_execution and add the motion planning plugin to the Rviz.

