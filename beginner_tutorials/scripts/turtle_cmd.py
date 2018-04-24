#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from beginner_tutorials.srv import *
from beginner_tutorials.msg import *
from geometry_msgs.msg import Twist
from math import pi,pow,sqrt,atan2

class Turtle:
    #Initialize the objects with the parameter declared here
    def __init__(self):
        #Initiate the node called Master_Turtle with a random value associated
        rospy.init_node('Master_Turtle', anonymous=True)
        #create a variable self.path that will publish in turtle1/cmd_vel
        #the message type is Twist
        self.path = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        #create a variable self.pose_acquite that will subscribe (read), the
        #message type pose in the topic /turtle1/pose through the self.callback
        self.pose_acquire = rospy.Subscriber('/turtle1/pose',Pose,self.callback)
        #declare self.speed as Twist type
        self.speed = Twist()
        #declare self.position as Pose type
        self.position = Pose()
        #delcare the self.rate with a rate of 10 Hz
        self.rate = rospy.Rate(10)

    #Function responsable for acquiring the data from /turtle1/pose
    def callback(self, data):
        #self.position receives the data from /turtle1/pose
        self.position = data
        #rounds the value in salf.position.x to 6 decimals
        self.position.x = round(self.position.x,6)
        #rounds the value in salf.position.y to 6 decimals
        self.position.y = round(self.position.y,6)

    #Interface that shows the possible actions and allows the user to choose
    #which possible action they want
    def select_action(self):
        #Shows the possible actions available and they corresponding value for
        #input
        print "\nTo move in straingt line press 1"
        print "To rotate the turtle press 2"
        print "to move to target press 3"
        print "to move to target, simple version, press 4"
        print "to quit press 5"
        #Receiveing the user's input
        choice = input("Insert here your choice: ")
        #analise the choice made by the user and direct to the corresponding
        #function
        if choice == 1:
            self.move_straight()
        elif choice == 2:
            self.move_rotate()
        elif choice == 3:
            self.move_to_target()
        elif choice == 4:
            self.move_to_target_simple_version()
        elif choice == 5:
            sys.exit(1)
        #else is here in case the user types something different than allowed
        else :
            print "Wrong input command! Moving back to selection panel"
            self.select_action()

    #Function that moves the turtle in straight line
    def move_straight(self):
        #Receiveing the user's input
        print("\nLet's Move")
        speed = input("Inform the speed:")
        distance = input("Inform the distance:")
        x = input("Foward (1) or Backward (0)?: ")
        #Analise if the user wants to move forward or backward
        if x == 1:
            print "Returning: moving forward"
            self.speed.linear.x = speed
        elif x == 0:
            print "Returning: moving backward"
            self.speed.linear.x = -speed
        #else is here in case the user types something different than allowed
        else:
            print "Wrong input command! Moving back to selection panel"
            self.select_action()
        #Only self.speed.linear.x is necessary to move in straight line so here
        #the other values of self.speed are set to 0
        self.speed.linear.y = 0
        self.speed.linear.z = 0
        self.speed.angular.x = 0
        self.speed.angular.y = 0
        self.speed.angular.z = 0
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        #sets the current_distance traveled as 0
        current_distance = 0
        #Loop to move the turtle in an specified distance set by user
        while(current_distance < distance):
            #Publish the velocity
            self.path.publish(self.speed)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distance already traveled
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        #set self.speed.linear.x to 0
        self.speed.linear.x = 0
        #Publish the velocity with self.speed.linear.x as 0 to stop the robot
        self.path.publish(self.speed)
        #Print to inform the user that they will go back to selection panel
        print "You can go back to selection a new action"
        #Moves back to selection panel to allow the user to take new actions
        self.select_action()

    #Function that rotates the turtle
    def move_rotate(self):
        #Receiveing the user's input
        print "\nLets Move this Turtle"
        speed = input("Inform speed of rotation (degrees/sec): ")
        angle = input("Inform the angle of rotation (degrees): ")
        direction = input("Inform if is clockwise (0) or anti-clockwise(1):")
        #Pass the speed in degrees/sec to radians/sec
        turn_speed = speed*2*pi/360
        #pass the angle of rotation in degrees to radians
        final_angle = angle*2*pi/360
        #Analise if the user wants to move clockwise or anti-clockwise
        if direction == 1:
            print "Returning: moving anti-clockwise"
            self.speed.angular.z = turn_speed
        elif direction == 0:
            print "Returning: moving clockwise"
            self.speed.angular.z = -turn_speed
        #else is here in case the user types something different than allowed
        else:
            print "Wrong input command! Moving back to selection panel"
            self.select_action()
        #Only self.speed.angular.z is necessary to rotate so here the other
        #values of self.speed are set to 0
        self.speed.linear.x = 0
        self.speed.linear.y = 0
        self.speed.linear.z = 0
        self.speed.angular.x = 0
        self.speed.angular.y = 0
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        #sets the current_angle rotated as 0
        current_angle = 0
        #Loop to move the turtle in an specified rotation informed by the user
        while(current_angle < final_angle):
            #Publish the velocity
            self.path.publish(self.speed)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates current angle rotated
            current_angle= turn_speed*(t1-t0)
        #After the loop, stops the robot
        #set self.speed.angular.z to 0
        self.speed.angular.z = 0
        #Publish the velocity with self.speed.angular.z as 0 to stop the robot
        self.path.publish(self.speed)
        #Print to inform the user that they will go back to selection panel
        print "You can go back to selection a new action"
        #Moves back to selection panel to allow the user to take new actions
        self.select_action()

    #Function that moves the turtle to a specified point given by the user
    #using a proportional controller
    def move_to_target(self):
        #set a variable that will receive the coordinates from the user as a
        #Pose variable
        target_goal = Pose()
        #Receiveing the user's input
        print ("\nLets Move")
        target_goal.x = input("tell me the x goal: ")
        target_goal.y = input("tell me the y goal: ")
        tolerance = input("Insert the tolerance: ")
        #In case the user types a tolerance smaller than the minimum value set
        #the tolerance as the minumum value allowed
        if tolerance < 0.000001:
            print ("Be more tolerant, please")
            tolerance = 0.000001
            print("Setting tolerance to 0.000001")
        #Only self.speed.angular.z and self.speed.linear.x is necessary to
        #rotate so here the other values of self.speed are set to 0
        self.speed.linear.y = 0
        self.speed.linear.z = 0
        self.speed.angular.x = 0
        self.speed.angular.y = 0
        #Inform that the turtle will start moving
        print ("Moving to Target!")

        #calulates the actual_distance to target, because the expression is too
        #long it was seperated in two lines
        actual_distance = sqrt(pow((target_goal.x - self.position.x), 2) + \
        pow((target_goal.y - self.position.y), 2))

        #calulates the angular_value to target, because the expression is too
        #long it was seperated in two lines
        angular_value = (atan2(target_goal.y - self.position.y,target_goal.x - \
        self.position.x) - self.position.theta)

        #Loob to get to target until the difference to target is smaller than
        #the tolerance established by the user
        while (actual_distance >= tolerance):
            #Set the linear speed as the actual_distance with a proportional of
            #1.5
            self.speed.linear.x = 1.5*actual_distance
            #set the angular speed as the actual angular value with a
            #proportional of 4
            self.speed.angular.z = 4*angular_value
            #Publish the speed
            self.path.publish(self.speed)

            #Get the new value of the actual distance to target, because the
            #expression is too long it was seperated in two lines
            actual_distance = sqrt(pow((target_goal.x - self.position.x), 2) + \
            pow((target_goal.y - self.position.y), 2))

            #Get the new value of angular value to target , because the
            #expression is too long it was seperated in two lines
            angular_value = (atan2(target_goal.y - self.position.y, \
            target_goal.x - self.position.x) - self.position.theta)

            #Make the code wait to ensure the rate of 10Hz is respected
            self.rate.sleep()
        #After the loop, stops the robot
        #set self.speed.linear.x to 0
        self.speed.linear.x = 0
        #set self.speed.angular.z to 0
        self.speed.angular.z = 0
        #Publish the velocity with self.speed.angular.z and self.speed.linear.x
        #as 0 to stop the robot
        self.path.publish(self.speed)
        #Print to inform the user that they will go back to selection panel
        print "You can go back to selection a new action"
        #Moves back to selection panel to allow the user to take new actions
        self.select_action()

    #Function that moves the turtle to a specified point given by the user
    #is a simply version of move_to_target without the proportional controller
    #that first rotates the turtle and then moves in a straight linear, is not
    #as good as move_to_target because we cant successfully control the error
    def move_to_target_simple_version(self):
        #set a variable that will receive the coordinates from the user as a
        #Pose variable
        target_goal = Pose()
        #Receiveing the user's input, note that we dont ask for the tolerance
        #value in this case because we cant successfully control it
        print ("\nLets Move")
        target_goal.x = input("tell me the x goal: ")
        target_goal.y = input("tell me the y goal: ")
        #As mention above first we will just rotate the turtle and after we will
        #move it in a straight line

        #Only self.speed.angular.z is necessary to rotate so here the other
        #values of self.speed are set to 0
        self.speed.linear.x = 0
        self.speed.linear.y = 0
        self.speed.linear.z = 0
        self.speed.angular.x = 0
        self.speed.angular.y = 0
        #calulates the angular_value to target, because the expression is too
        #long it was seperated in two lines
        angular_value = (atan2(target_goal.y - self.position.y,target_goal.x - \
        self.position.x) - self.position.theta)
        #Analise if the variable angular_value has a value below 0, if there is
        #we rotate clockwise and if is above 0 we move anti-clockwise, note that
        #the speed we both cases is set as pi/2 and is not a parameter set by
        #the user
        if angular_value >= 0:
            self.speed.angular.z = pi/2
        else:
            self.speed.angular.z = -pi/2
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        #sets the current_angle rotated as 0
        current_angle = 0
        #Loop to move the turtle in an specified rotation informed by the user,
        #we use abs(angular_value) because angular_value can have both positive
        #or negative values but we want avaliate the distance(abs) between
        #current_angle and angular_value
        while(current_angle < abs(angular_value)):
            #Publish the velocity
            self.path.publish(self.speed)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates current angle rotated
            current_angle= pi/2*(t1-t0)
        #After the loop, stops the robot
        #set self.speed.angular.z to 0
        self.speed.angular.z = 0
        #Publish the velocity with self.speed.angular.z as 0 to stop the robot
        self.path.publish(self.speed)
        #Now that we rotate the turtle in order to "face" the point we want to
        #move, we will move in a straight line

        #Calculates the distance to target
        distance = sqrt(pow((target_goal.x - self.position.x), 2) + \
        pow((target_goal.y - self.position.y), 2))
        #set the self.speed.linear.x to 10,note that again the speed we both
        #cases is set as 10 and is not a parameter set by the user and because
        # we already pass the other values of self.speed as 0 we dont need to
        #do it again
        self.speed.linear.x = 10
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        #sets the current_distance traveled as 0
        current_distance = 0
        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            self.path.publish(self.speed)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distance already traveled
            current_distance= 10*(t1-t0)
        #After the loop, stops the robot
        #set self.speed.linear.x to 0
        self.speed.linear.x = 0
        #Publish the velocity with self.speed.linear.x as 0 to stop the robot
        self.path.publish(self.speed)
        #Print to inform the user that they will go back to selection panel
        print "You can go back to selection a new action"
        #Moves back to selection panel to allow the user to take new actions
        self.select_action()

if __name__ == '__main__':
        #Creates a object called turtle10 of the class Turtle
        turtle10 = Turtle()
        #Calls the select_action function of the class to allow the user to
        #choose new actions
        turtle10.select_action()
