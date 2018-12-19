#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from math import pi
from std_msgs.msg import Float64
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from simulation_control.srv import LineMovement

#!/usr/bin/env python

class elir_odometry():
    """
    Class to calculate the elir odometry and implement the service to an specific line displacement
    @param: Empty req  Empty Request
    """
    def __init__(self):
        #Initiating odometry node
        rospy.init_node("elir_line__movement_service")

        self.joint_state_subs = rospy.Subscriber('elir/joint_states',JointState, self.state_callback)
        
        self.traction_f_publisher = rospy.Publisher('elir/traction_f_controller/command',Float64,queue_size = 10)
        self.traction_ap_publisher = rospy.Publisher('elir/traction_ap_controller/command', Float64,queue_size = 10)
        self.traction_b_publisher = rospy.Publisher('elir/traction_b_controller/state', Float64,queue_size = 10)

        self.line_move_service = rospy.Service('line_movement/', LineMovement, self.line_movement)

        self.odometry_data = Odometry()
        self.odometry_final = 0
        self.linear_vel_final = 0

        #Odometry value
        self.odometry_tr1 = 0.0
        self.odometry_tr2 = 0.0
        #previous_time_tr1 used in delta calculus
        self.previous_time_tr1 = 0.0
        self.previous_time_tr2 = 0.0
        #Variable auxiliar to acess a data from one callback function in the other
        self.odometry_aux = 0.0
        self.velocity_aux = 0.0

        self.speed = 3

        rospy.spin()

    def line_movement(self,req):
        self.odometry_data = 0

        if req.displacement > 0:
            self.vel_publish(self.speed)
        
        elif req.displacement == 0:
            return True
        
        else:
            self.vel_publish(-self.speed)

        while(req.displacement > self.odometry_data):
            rospy.loginfo("Caminhado:",self.odometry,"/",req.displacement)

        self.vel_publish(0)
        rospy.loginfo("Robot stopped")
    
    #Plubishes the velocity value in the traction topics
    def vel_publish(self,value):
        self.traction_b_publisher.publish(value)
        self.traction_f_publisher.publish(value)
        self.traction_ap_publisher.publish(-value)

    #Callback function implementing the JointState received
    def state_callback(self, data):
        for i in data:
            print(i)


        angular_vel = data.velocity
        #Calculating linear velocity using the wheel radius
        linear_vel = angular_vel*self.wheel_radius
        #Estimating time to delta calculus for integration
        motor_time_sec = data.header.stamp.secs
        motor_time_nsec = data.header.stamp.nsecs*1e-9
        motor_time = motor_time_sec + motor_time_nsec

        delta_time = motor_time - self.previous_time_tr1
        #Calculating the distance diferential
        self.position_tr1_variation = linear_vel*delta_time
        #Implementing in the total distance
        self.odometry_tr1 = self.odometry_tr1 + self.position_tr1_variation
        #Reseting time to next distance diferential calculus
        self.previous_time_tr1 = motor_time
        #Calculating the final odometry_tr1
        if self.controllers_qtd == 1:
            odometry_final  = self.odometry_tr1
            linear_vel_final = linear_vel
        else:
            odometry_final = (self.odometry_tr1 + self.odometry_aux)/2
            linear_vel_final = (linear_vel + self.velocity_aux)/2

        self.odometry_data.pose.pose.position.x = self.odometry_final
        self.odometry_data.twist.twist.linear.x = self.linear_vel_final
        self.odometry_data.header.stamp = rospy.Time.now()
        
    def traction2_callback(self, data):
        #Due do the diferent spin direction of the servos, whe have to:
        angular_vel = -data.velocity
        #Calculating linear velocity using the wheel radius
        linear_vel = angular_vel*self.wheel_radius
        #Estimating time to delta calculus for integration
        motor_time_sec = data.header.stamp.secs
        motor_time_nsec = data.header.stamp.nsecs
        motor_time = motor_time_sec + motor_time_nsec*1e-9

        delta_time = motor_time - self.previous_time_tr2
        #Calculating the distance diferential
        self.position_tr2_variation = linear_vel*delta_time
        #Implementing in the total distance
        self.odometry_tr2 = self.odometry_tr2 + self.position_tr2_variation
        #Reseting time to next distance diferential calculus
        self.previous_time_tr2 = motor_time
        #Auxiliar variable for distance media
        self.odometry_aux = self.odometry_tr2
        #Auxiliar variable for velocity media
        self.velocity_aux = linear_vel

if __name__ == '__main__':

    x = elir_odometry()
