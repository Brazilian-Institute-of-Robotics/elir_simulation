#!/usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        
        rospy.init_node('trajectory_client')
        
        
        arm_joints = ['joint1_f',
                      'joint2_f',
                      'joint1_t',
                      'joint2_t']
                      
        arm_goal = [-0.2, -0.3, -0.2, -0.3]

        arm_client =actionlib.SimpleActionClient('robot/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        arm_client.wait_for_server()

        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)

        arm_goal = FollowJointTrajectoryGoal()
        
        arm_goal.trajectory = arm_trajectory
        
        arm_goal.goal_time_tolerance  = rospy.Duration(2.0)
        
        arm_client.send_goal(arm_goal)
        
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        robot_trajectory = TrajectoryDemo() 
       
    except rospy.ROSInterruptException:
        print("deu merda")
        
