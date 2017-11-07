#!/usr/bin/env python
from __future__ import print_function
import rospy
# Brings in the SimpleActionClient
import actionlib
import beginner_tutorials.msg

class pow_client:
    #Initialize the objects with the parameter declared here
    def __init__(self):
        self.client = actionlib.SimpleActionClient('two_server', beginner_tutorials.msg.powAction)

    def two_pow(self):

        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = beginner_tutorials.msg.powGoal(order = input("Declare the order: "))

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        # Prints out the result of executing the action
        print (self.client.get_result())  # A FibonacciResult

if __name__ == '__main__':
    try:
        rospy.init_node('two_pow_client_py')
        result = pow_client()
        result.two_pow()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
