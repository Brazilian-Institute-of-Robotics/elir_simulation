#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from math import pi,pow,sqrt

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



        self.joint_1f_path = rospy.Publisher ('/robot/joint_1f_controller/command', Float64, queue_size=10)

        self.joint_1b_path = rospy.Publisher ('/robot/joint_1b_controller/command', Float64, queue_size=10)

        self.joint_2f_path = rospy.Publisher ('/robot/joint_2f_controller/command', Float64, queue_size=10)

        self.joint_2b_path = rospy.Publisher ('/robot/joint_2b_controller/command', Float64, queue_size=10)

        self.claw_1f_path = rospy.Publisher ('/robot/claw_1f_controller/command', Float64, queue_size=10)
        self.claw_2f_path = rospy.Publisher ('/robot/claw_2f_controller/command', Float64, queue_size=10)
        self.claw_1b_path = rospy.Publisher ('/robot/claw_1b_controller/command', Float64, queue_size=10)
        self.claw_2b_path = rospy.Publisher ('/robot/claw_2b_controller/command', Float64, queue_size=10)
        self.claw_1c_path = rospy.Publisher ('/robot/claw_1c_controller/command', Float64, queue_size=10)


        self.hokuyo_path = rospy.Subscriber ('/robot/scan', LaserScan,    self.callback_hokuyo)
        #variable that contains the speed of the robot
        self.speed = Float64()
        #Initialize the speed as 2
        self.speed = 15

        #Variable that contains the duration of movement and initializa as 3 secondes
        self.time = 3
        self.hokuyu_info = Float32()
        self.hojuyu_inc_angle = Float32()
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
        print "To move until find, approx. and surpass an obstacle press 5"
        print "To quit press 6"
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
            self.detect_obs()
        if choice == 6:
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
        self.traction_1f_path.publish(-self.speed)
        self.traction_2f_path.publish(self.speed)
        self.traction_1b_path.publish(-self.speed)
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
        self.traction_1f_path.publish(self.speed)
        self.traction_2f_path.publish(-self.speed)
        self.traction_1b_path.publish(self.speed)
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

    def callback_hokuyo(self, data):
        self.hokuyu_info = filter(lambda x: x != float('Inf'), data.ranges)
        self.hojuyu_inc_angle = data.angle_increment

    # def hokuyo_OLDER_VERSIONS(self):
    #     largura = 0
    #     while largura*1000 < 30:
    #         print "Obstacle not detected, proceding!"
    #         length = len(self.hokuyu_info)
    #         pos_ini = Float32()
    #         pos_final = Float32()
    #         lock = 1
    #         soma = 0
    #         for pos in range(0,(length)):
    #             if self.hokuyu_info[pos] != float('Inf'):
    #                 soma = soma + self.hokuyu_info[pos]
    #                 if lock == 1:
    #                     pos_ini = pos
    #                     lock = 0
    #             else:
    #                 if lock == 0:
    #                     pos_final = pos - 1
    #                     break
    #         media = soma/(pos_final-pos_ini+1)
    #         largura = media*self.hojuyu_inc_angle* ((pos_final-pos_ini)+1)
    #         print "largura lida em mm:", largura*1000
    #         self.traction_1c_path.publish(self.speed)
    #         self.traction_1f_path.publish(self.speed)
    #         self.traction_2f_path.publish(self.speed)
    #         self.traction_1b_path.publish(self.speed)
    #         self.traction_2b_path.publish(self.speed)
    #     print "Obstacle detected, preparing to aprox. it"
    #     self.traction_1c_path.publish(0)
    #     self.traction_1f_path.publish(0)
    #     self.traction_2f_path.publish(0)
    #     self.traction_1b_path.publish(0)
    #     self.traction_2b_path.publish(0)
    #     self.aprox_obs(sqrt(pow(media,2)-pow(0.4,2)))
    #     print"Moving back to panel"
    #     self.IHM_Panel()

    def detect_obs(self):
        largura = 0
        while largura*1000 < 30:
            print "Obstacle not detected, proceding!"
            soma = 0
            hokuyu_info = self.hokuyu_info
            length = len(hokuyu_info)
            for pos in range(0,(length)):
                soma = soma + hokuyu_info[pos]
            largura = self.hojuyu_inc_angle*soma
            print "largura lida em mm:", largura*1000
            self.traction_1c_path.publish(self.speed)
            self.traction_1f_path.publish(-self.speed)
            self.traction_2f_path.publish(self.speed)
            self.traction_1b_path.publish(-self.speed)
            self.traction_2b_path.publish(self.speed)
        print "Obstacle detected, preparing to aprox. it"
        self.traction_1c_path.publish(0)
        self.traction_1f_path.publish(0)
        self.traction_2f_path.publish(0)
        self.traction_1b_path.publish(0)
        self.traction_2b_path.publish(0)
        self.aprox_obs(sqrt(pow(soma/length,2)-pow(0.4,2)))
        print"Finished all movements, Moving back to panel"
        self.IHM_Panel()

    def aprox_obs(self,dist):
        speed = 5
        time = ((dist-0.08)/speed)*60
        self.traction_1c_path.publish(speed)
        self.traction_1f_path.publish(-speed)
        self.traction_2f_path.publish(speed)
        self.traction_1b_path.publish(-speed)
        self.traction_2b_path.publish(speed)
        current_time = 0
        starting_time = rospy.get_time()
        # print starting_time
        # x = 0
        while current_time < time:
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
        print "aprrox. completed, preparing to surpass it"
        self.surpass_obs()

    def surpass_obs(self):
        #move up front arm to allow claws to rotate
        self.joint_1f_path.publish(-0.3)
        self.joint_2f_path.publish(0.1)
        #Create a delay, to allow the arm to go up before rotating claws
        time = 1
        current_time = 0
        starting_time = rospy.get_time()
        while current_time < time:
            actual_time = rospy.get_time()
            current_time = actual_time - starting_time
        #Rotate claws
        self.claw_1f_path.publish(3.14)
        self.claw_2f_path.publish(3.14)
        #Move some distance to allow front arm to pass the obstacle
        dist = 0.6
        speed = 5
        time = ((dist-0.08)/speed)*60
        current_time = 0
        starting_time = rospy.get_time()
        self.traction_1c_path.publish(speed)
        self.traction_1f_path.publish(-speed)
        self.traction_2f_path.publish(speed)
        self.traction_1b_path.publish(-speed)
        self.traction_2b_path.publish(speed)
        while current_time < time:
            actual_time = rospy.get_time()
            current_time = actual_time - starting_time
        self.traction_1c_path.publish(0)
        self.traction_1f_path.publish(0)
        self.traction_2f_path.publish(0)
        self.traction_1b_path.publish(0)
        self.traction_2b_path.publish(0)
        #rotate the claws back to original position
        self.claw_1f_path.publish(0)
        self.claw_2f_path.publish(0)
        #small delay to give sufficient time to claws rotate back to position
        time = 0.5
        current_time = 0
        starting_time = rospy.get_time()
        while current_time < time:
            actual_time = rospy.get_time()
            current_time = actual_time - starting_time
        #move front arm back to original position
        self.joint_1f_path.publish(0)
        self.joint_2f_path.publish(0)
        #move both arms down to force the center up
        self.joint_1f_path.publish(1.3)
        self.joint_1b_path.publish(1.3)
        #Create a delay, to allow the center to go up before rotating claw
        time = 1
        current_time = 0
        starting_time = rospy.get_time()
        while current_time < time:
            actual_time = rospy.get_time()
            current_time = actual_time - starting_time
        #Rotate center claw
        self.claw_1c_path.publish(3.14)
        #Move some distance to allow the center to pass the obstacle
        dist = 0.6
        speed = 5
        time = ((dist-0.08)/speed)*60
        current_time = 0
        starting_time = rospy.get_time()
        self.traction_1c_path.publish(speed)
        self.traction_1f_path.publish(-speed)
        self.traction_2f_path.publish(speed)
        self.traction_1b_path.publish(-speed)
        self.traction_2b_path.publish(speed)
        while current_time < time:
            actual_time = rospy.get_time()
            current_time = actual_time - starting_time
        self.traction_1c_path.publish(0)
        self.traction_1f_path.publish(0)
        self.traction_2f_path.publish(0)
        self.traction_1b_path.publish(0)
        self.traction_2b_path.publish(0)
        #Rotate the center claw back to orginal position and arms back to positio to lower the center
        self.claw_1c_path.publish(0)
        self.joint_1f_path.publish(0)
        self.joint_1b_path.publish(0)
        #move up back arm to allow claws to rotate
        self.joint_1b_path.publish(-0.3)
        self.joint_2b_path.publish(0.1)
        #Create a delay, to allow the arm to go up before rotating claws
        time = 1
        current_time = 0
        starting_time = rospy.get_time()
        while current_time < time:
            actual_time = rospy.get_time()
            current_time = actual_time - starting_time
        #Rotate claws
        self.claw_1b_path.publish(3.14)
        self.claw_2b_path.publish(3.14)
        #Move some distance to allow front arm to pass the obstacle
        dist = 0.6
        speed = 5
        time = ((dist-0.08)/speed)*60
        current_time = 0
        starting_time = rospy.get_time()
        self.traction_1c_path.publish(speed)
        self.traction_1f_path.publish(-speed)
        self.traction_2f_path.publish(speed)
        self.traction_1b_path.publish(-speed)
        self.traction_2b_path.publish(speed)
        while current_time < time:
            actual_time = rospy.get_time()
            current_time = actual_time - starting_time
        self.traction_1c_path.publish(0)
        self.traction_1f_path.publish(0)
        self.traction_2f_path.publish(0)
        self.traction_1b_path.publish(0)
        self.traction_2b_path.publish(0)
        #move claws and arm back to original position
        self.claw_1b_path.publish(0)
        self.claw_2b_path.publish(0)
        self.joint_1b_path.publish(0)
        self.joint_2b_path.publish(0)
        print "Surpassed"

if __name__ == '__main__':
        #Creates a object called turtle10 of the class Turtle
        piro3 = Robot()
