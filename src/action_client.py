#! /usr/bin/env python

import rospy
import actionlib


from com2009_actions.msg import SearchAction, SearchGoal, CameraSweepAction, CameraSweepGoal
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
from geometry_msgs.msg import Twist

class action_client:

    def feedback_callback(self, feedback_data):
        self.distance = feedback_data.current_distance_travelled
        if self.count > 100:
            #print('FEEDBACK: current_distance_travelled: {}'.format(feedback_data.current_distance_travelled))
            self.count = 0 
        else: 
            self.count += 1

    def shutdown_ops(self):
        # publish an empty twist message to stop the robot (by default all 
        # velocities within this will be zero):
        if not self.action_complete :
            self.robot_controller.stop()
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
        
    def __init__(self):
        rospy.init_node('search_action_client')

        self.client = actionlib.SimpleActionClient('/search_action_server', 
                    SearchAction)
        self.client.wait_for_server()
    
        self.rate = rospy.Rate(1)
        self.action_complete = False
        self.count = 0
        self.distance = 0 
        self.goal = SearchGoal()
    
        rospy.on_shutdown(self.shutdown_ops)
    
    def send_goal(self, velocity, approach):

        self.goal.fwd_velocity = velocity

        self.goal.approach_distance = approach

        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def check_hit(self, d):
        global go
        if self.front_round_distance <= d or self.back_round_distance <= d:
            print("front or back hit client")
            rospy.loginfo('Cancelling the move.')
            self.actionserver.set_preempted()
            # stop the robot:
            self.robot_controller.stop()
            success = False
            go = False
            sys.exit()
            return True
        elif self.back_round_distance <= d:
            print("back hit client")
            rospy.loginfo('Cancelling the move.')
            self.actionserver.set_preempted()
            # stop the robot:
            self.robot_controller.stop()
            success = False
            go = False
            sys.exit()
            return True


    def main(self):
        self.send_goal(0.2, 0.7)
        while self.client.get_state() < 2:
            if self.distance >= 200:
                self.client.cancel_goal()
                break

            self.rate.sleep()

        #print("RESULT: {}".format(self.client.get_result()))
        self.action_complete = True



if __name__ == '__main__':
    ac_object = action_client()
    try:
        ac_object.main()
    except rospy.ROSInterruptException:
        pass