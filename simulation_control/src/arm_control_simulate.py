#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class arm_mimic_control():
"""
THis class is created to configures the subscribers and publisher of joints and teleop keys, also the parameters
of the back arm joints and front arm joints. It contais the methods to get the joints positions and teleop values
to convert and publish on both arms joints.
"""
    def __init__(self):
  	"""
  	Method to create and configures joints and nodes to publish on ROS framework
  	"""
        #Creating our node,publisher and subscriber
        self.key_subscriber = rospy.Subscriber('/key_vel', Twist, self.callback)
        self.b_arm_publisher = rospy.Publisher('/elir/b_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.f_arm_publisher = rospy.Publisher('/elir/f_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.joints_subscriber = rospy.Subscriber('/elir/joint_states', JointState, self.callback_joints) 
        self.f_arm_joints = ['joint1_f', 'joint2_f']
        self.b_arm_joints = ['joint1_b', 'joint2_b']
        self.trajectory_duration = rospy.rostime.Duration(0.8)
        rospy.spin()

    #Callback function implementing the pose value received
    def callback_joints(self, msg):
  	"""
  	Method to get joints positions 
	: msg JointState:	Variable with joints positions informations
  	"""
        self.current_joint1b = msg.position[2]
        self.current_joint1f = msg.position[3] 

    def callback(self, data):
  	"""
  	Method to get the value published on key_vel topic, to realize a conversion an publish on joints
	:data Float64: 		Value to be converted and published on joints
  	"""
        #Verifies if is a null value on angular z
        if data.angular.z == 0:
            pass
	#Verifies the joints position
        if(self.current_joint1b == 0 and self.current_joint1f == 0):       
            pass
       
        time = rospy.get_time()
        #Create messages to publish on b_arm joints and configures b_arm parameters
        msg_to_b_arm = JointTrajectory()                
        msg_to_b_arm.joint_names = self.b_arm_joints
        msg_to_b_arm.points = []        
        points_to_b_arm = JointTrajectoryPoint()   
        points_to_b_arm.time_from_start = self.trajectory_duration  
        #Create messages to publish on f_arm joints and configures f_arm parameters        
	msg_to_f_arm = JointTrajectory()                   
        msg_to_f_arm.joint_names = self.f_arm_joints    
        msg_to_f_arm.points = []                     
        points_to_f_arm = JointTrajectoryPoint()    
        points_to_f_arm.time_from_start = self.trajectory_duration 
	#Converts the value of data of key_vel  
        key_vel = self.current_joint1b + 0.3 * data.angular.z  
	#Creates the list of points to publish
        points_to_f_arm.positions = [key_vel, 0.0]
        points_to_b_arm.positions = [key_vel, 0.0]
	#Update the list of points
        msg_to_b_arm.points.append(points_to_b_arm)
        msg_to_f_arm.points.append(points_to_f_arm)
        #Verifies the limits of key_vel value        
        if(key_vel >= 30 or key_vel <= -30):
	    #Value to publish don't change	
            key_vel = self.current_joint1b
            #Publish key_vel value on joints
            self.b_arm_publisher.publish(msg_to_b_arm)
            self.f_arm_publisher.publish(msg_to_f_arm) 
        else:
            #Publish key_vel value on joints
            self.b_arm_publisher.publish(msg_to_b_arm)
            self.f_arm_publisher.publish(msg_to_f_arm)

if __name__ == '__main__':
    try:
        rospy.init_node('elir_line_controller', anonymous=True)
        x = arm_mimic_control()
    except rospy.ROSInterruptException: pass
