#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
# from beginner_tutorials.srv import *
from beginner_tutorials.msg import *
from geometry_msgs.msg import Twist
from math import pi,pow,sqrt,atan2
from sensor_msgs.msg import Joy

class Turtle:
    #Initialize the objects with the parameter declared here
    def __init__(self):
        #Initiate the node called Master_Turtle with a random value associated
        rospy.init_node('Master_bot_Joy', anonymous=True)
        #create a variable self.path that will publish in turtle1/cmd_vel
        #the message type is Twist
        self.pathmove = rospy.Publisher('/robot/joint_1f_controller/command', Float64, queue_size=10)
        self.pathmove2 = rospy.Publisher('/robot/joint_1b_controller/command', Float64, queue_size=10)
        self.pathmove3 = rospy.Publisher('/robot/joint_2f_controller/command', Float64, queue_size=10)
        self.pathmove4 = rospy.Publisher('/robot/joint_2b_controller/command', Float64, queue_size=10)
        self.path = rospy.Publisher('/robot/traction_1f_controller/command', Float64, queue_size=10)
        self.path2 = rospy.Publisher('/robot/traction_2f_controller/command', Float64, queue_size=10)
        self.path3 = rospy.Publisher('/robot/traction_1b_controller/command', Float64, queue_size=10)
        self.path4 = rospy.Publisher('/robot/traction_2b_controller/command', Float64, queue_size=10)
        self.path5 = rospy.Publisher('/robot/traction_1c_controller/command', Float64, queue_size=10)
        self.pathclaw = rospy.Publisher('/robot/claw_1f_controller/command', Float64, queue_size=10)
        self.pathclaw2 = rospy.Publisher('/robot/claw_2f_controller/command', Float64, queue_size=10)
        self.pathclaw3 = rospy.Publisher('/robot/claw_1b_controller/command', Float64, queue_size=10)
        self.pathclaw4 = rospy.Publisher('/robot/claw_2b_controller/command', Float64, queue_size=10)
        self.pathclaw5 = rospy.Publisher('/robot/claw_1c_controller/command', Float64, queue_size=10)
        #self.path2 = rospy.Publisher('/robot/claw_2f_controller/command', Float64, queue_size=10)
        # self.path = rospy.Publisher('/robot/joint_2f_controller/command', Twist, queue_size=10)
        # self.path2 = rospy.Publisher('elir/2/cmd_vel', Twist, queue_size=10)
        #create a variable self.pose_acquite that will subscribe (read), the
        #message type pose in the topic /turtle1/pose through the self.callback
        self.pose_acquire = rospy.Subscriber('joy',Joy,self.callback)
        #declare self.speed as Twist type
        self.speed = Float64()
        self.movef = Float64()
        self.movef2 = Float64()
        self.moveb = Float64()
        self.moveb2 = Float64()
        self.clawf = Float64()
        self.clawm = Float64()
        self.clawb = Float64()
        #self.speed = Twist()
        self.speed2 = Twist()
        #declare self.position as Pose type
        self.joystick = Joy()
        #delcare the self.rate with a rate of 10 Hz
        self.rate = rospy.Rate(20)

    #Function responsable for acquiring the data from /turtle1/pose
    def callback(self, data):
        #self.joystick receives the data from /joy
        # self.joystick = data
        # self.speed.data = 2*self.joystick.axes[1]
        # self.speed.data = round(self.speed.data,0)
        # self.path.publish(self.speed)
        # self.speed2.data = self.joystick.axes[0]
        # self.speed2.dcmd_velata = round(self.speed2.data,0)
        # self.path2.publish(self.speed2)
        # self.rate.sleep()
        self.joystick = data
        if self.joystick.buttons[0] == 0:
            self.speed = 0
        else:
            self.speed = 4*self.joystick.axes[2]
        self.movef2 = 4*self.joystick.axes[0]
        self.movef = 4*self.joystick.axes[1]
        self.moveb2 = 4*self.joystick.axes[3]
        self.moveb = 4*self.joystick.axes[4]
        self.clawb = 4*self.joystick.buttons[1]
        self.clawm = 4*self.joystick.buttons[3]
        self.clawf = 4*self.joystick.buttons[2]
        # self.speed.linear.x = 6*self.joystick.axes[1]
        # self.speed.angular.z = -6*self.joystick.axes[0]
        #self.speed2 = 10*self.joystick.axes[1]
        # self.speed2.angular.z = -1*self.joystick.axes[0]
        self.pathmove.publish(-self.movef)
        self.pathmove2.publish(-self.moveb)
        self.pathmove3.publish(self.movef2)
        self.pathmove4.publish(-self.moveb2)
        self.path.publish(-self.speed)
        self.path2.publish(self.speed)
        self.path3.publish(-self.speed)
        self.path4.publish(self.speed)
        self.path5.publish(self.speed)
        self.pathclaw.publish(self.clawf)
        self.pathclaw2.publish(self.clawf)
        self.pathclaw3.publish(self.clawb)
        self.pathclaw4.publish(self.clawb)
        self.pathclaw5.publish(self.clawm)
        #self.path2.publish(self.speed2)
        self.rate.sleep()




if __name__ == '__main__':
        #Creates a object called turtle10 of the class Turtle
        turtle10 = Turtle()
        #Calls the select_action function of the class to allow the user to
        #choose new actions
        rospy.spin()
