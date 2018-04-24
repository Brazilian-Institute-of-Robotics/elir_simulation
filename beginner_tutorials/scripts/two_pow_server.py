#! /usr/bin/env python
import rospy
import actionlib
import beginner_tutorials.msg

class two_pow_server:

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, beginner_tutorials.msg.powAction, execute_cb=self.execute_cb, auto_start = False)
        self.feedback = beginner_tutorials.msg.powFeedback()
        self.result = beginner_tutorials.msg.powResult()
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(10)
        success = True

        # append the seeds for the fibonacci sequence
        self.feedback.sequence = []
        #self.feedback.sequence.append(0)
        self.feedback.sequence.append(1)

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i' % (self._action_name, goal.order, self.feedback.sequence[0]))

        # start executing the action
        for i in range(0, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self.feedback.sequence.append(self.feedback.sequence[i]*2)
            # publish the feedback
            self._as.publish_feedback(self.feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        if success:
            self.result.sequence = self.feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node('two_server')
    server = two_pow_server(rospy.get_name())
    rospy.spin()
