#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class Robot:
    def __init__(self):
        #initiate the note called auto_move_robot with a random number associated
        rospy.init_node('auto_move_robot', anonymous=True)

        #create the topic for shaft 1c
        self.traction_1c_path = rospy.Publisher ('/robot/traction_1c_controller/command', Float64, queue_size=10)

        #create the topic for shaft 1f
        self.traction_1f_path = rospy.Publisher ('/robot/traction_1f_controller/command', Float64, queue_size=10)

        #create the topic for shaft 2f
        self.traction_2f_path = rospy.Publisher ('/robot/traction_2f_controller/command', Float64, queue_size=10)

        #create the topic for shaft 1b
        self.traction_1b_path = rospy.Publisher ('/robot/traction_1b_controller/command', Float64, queue_size=10)

        #create the topic for shaft 2b
        self.traction_2b_path = rospy.Publisher ('/robot/traction_2b_controller/command', Float64, queue_size=10)

        #variable that contains the speed of the robot
        self.speed = Float64()
        #Initialize the speed as 2
        self.speed = 2

        #Variable that contains the duration of movement and initializa as 3 secondes
        self.time = 3

        #Go to panel for decision making
        self.IHM_Panel()

    def IHM_Panel(self):
        print "\nCurrent speed:", self.speed
        print "Duration of movement in sec:", self.time
        print "current time", rospy.Time.now().to_sec()
        print "\nTo move forward press 1"
        print "To move backwards press 2"
        print "To change the speed press 3"
        print "To change the duration of the movement press 4"
        print "To quit press 5"
        choice = input("\nInsert the number here: ")
        if choice == 1:
            self.move_forward()
        if choice == 2:
            self.move_backward()
        if choice == 3:
            self.change_speed()
        if choice == 4:
            self.change_time()
        if choice == 5:
            sys.exit(1)
        # if choice == None:
        #     print "Inserted an invalid number, going back to panel"
        #     self.IHM_Panel()
        else:
            print "Inserted an invalid number, going back to panel"
            self.IHM_Panel()

    def move_forward(self):
        print "\nMoving Forward!"
        self.traction_1c_path.publish(self.speed)
        self.traction_1f_path.publish(self.speed)
        self.traction_2f_path.publish(self.speed)
        self.traction_1b_path.publish(self.speed)
        self.traction_2b_path.publish(self.speed)
        current_time = 0
        starting_time = rospy.get_time()
        # print starting_time
        # x = 0
        while current_time < self.time:
            actual_time = rospy.get_time()
            current_time = actual_time - starting_time
            # if x == 0:
            #     print "actual time:", actual_time
            #     print "current time:", current_time
            #     x = 1
        self.traction_1c_path.publish(0)
        self.traction_1f_path.publish(0)
        self.traction_2f_path.publish(0)
        self.traction_1b_path.publish(0)
        self.traction_2b_path.publish(0)
        print "Moving back to panel"
        self.IHM_Panel()

    def move_backward(self):
        print "\nMoving backwards!"
        self.traction_1c_path.publish(-self.speed)
        self.traction_1f_path.publish(-self.speed)
        self.traction_2f_path.publish(-self.speed)
        self.traction_1b_path.publish(-self.speed)
        self.traction_2b_path.publish(-self.speed)
        current_time = 0
        starting_time = rospy.Time.now().to_sec()
        while current_time < self.time:
            actual_time = rospy.Time.now().to_sec()
            current_time = actual_time - starting_time
        self.traction_1c_path.publish(0)
        self.traction_1f_path.publish(0)
        self.traction_2f_path.publish(0)
        self.traction_1b_path.publish(0)
        self.traction_2b_path.publish(0)
        print "Moving back to panel"
        self.IHM_Panel()

    def change_speed(self):
        self.speed = abs(input("\nInsert new absolute value of the speed:"))
        print"Moving back to panel"
        self.IHM_Panel()

    def change_time(self):
        self.time = abs(input("\nInsert new duration time of movement:"))
        print"Moving back to panel"
        self.IHM_Panel()

if __name__ == '__main__':
        #Creates a object called turtle10 of the class Turtle
        piro3 = Robot()
