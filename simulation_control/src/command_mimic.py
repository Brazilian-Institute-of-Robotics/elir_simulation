#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist

class line_mimic_control():
    """Class to mimmic the key_vel topic to the traction control topics"""
    def __init__(self):
        #Creating our node,publisher and subscriber
        self.key_subscriber = rospy.Subscriber('/key_vel', Twist, self.callback)
        self.traction_f1_publisher = rospy.Publisher('elir/traction_f1_controller/command', Float64, queue_size=10)
        self.traction_f2_publisher = rospy.Publisher('elir/traction_f2_controller/command', Float64, queue_size=10)
        self.traction_b1_publisher = rospy.Publisher('elir/traction_b1_controller/command', Float64, queue_size=10)
        self.traction_b2_publisher = rospy.Publisher('elir/traction_b2_controller/command', Float64, queue_size=10)
        self.traction_ap_publisher = rospy.Publisher('elir/traction_ap_controller/command', Float64, queue_size=10)

        rospy.spin()
    #Callback function implementing the pose value received
    def callback(self, data):  
        """Callback which sends the key values to the traction units
        @param: Twist data Data containing a twist message type
        """
        key_vel = 30*data.angular.z
        self.traction_f1_publisher.publish(key_vel)
        self.traction_f2_publisher.publish(-key_vel)
        self.traction_b1_publisher.publish(-key_vel)
        self.traction_b2_publisher.publish(key_vel)
        self.traction_ap_publisher.publish(key_vel)

if __name__ == '__main__':
    try:
        rospy.init_node('simulation_line_controller', anonymous=True)
        x = line_mimic_control()
    except rospy.ROSInterruptException: pass