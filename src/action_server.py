#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_actions.msg import SearchFeedback, SearchResult, SearchAction

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import some other useful Python Modules
import math
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from math import sqrt
import sys

class ActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        self.rate = rospy.Rate(10)

        self.scanSub = rospy.Subscriber('scan', LaserScan, self.callback_function)

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        # define the robot pose variables and set them all to zero to start with:
        self.x0 = 0.0
        self.y0 = 0.0
        self.stop_distance = 0
        self.desired_velocity = 0
        self.turn_direction = False
        
        # define a Twist instance, which can be used to set robot velocities
        self.front_distance = 0.0
        self.front_angle = 0.0
        self.right_distance = 0.0
        self.right_angle = 0.0
        self.left_distance = 0.0
        self.left_angle = 0.0
        self.back_distance = 0.0
        self.back_angle = 0.0


    def callback_function(self, scan_data):

        # front detection
        front_left_arc = scan_data.ranges[0:16]
        front_right_arc = scan_data.ranges[-15:]
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angle = np.arange(-15, 16)

        # find the miniumum object distance within the frontal laserscan arc:
        self.front_distance = front_arc.min()
        self.front_angle = front_arc_angle[np.argmin(front_arc)]
        
        #right detection
        right_arc = scan_data.ranges[314:344]
        right_side_arc = np.array(right_arc[::1])
        right_arc_angle = np.arange(314,344)

        # find the miniumum object distance within the right laserscan arc:
        self.right_distance = right_side_arc.min()
        self.right_angle = right_arc_angle[np.argmin(right_side_arc)]


        #left detection
        left_arc = scan_data.ranges[16:46]
        left_side_arc = np.array(left_arc[::1])
        left_arc_angle = np.arange(16,46)

        # find the miniumum object distance within the left laserscan arc:
        self.left_distance = left_side_arc.min()
        self.left_angle = left_arc_angle[np.argmin(left_side_arc)]

    
    def action_server_launcher(self, goal):
        success = True
        x = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))

        # check goal
        if goal.fwd_velocity <= 0 or goal.fwd_velocity >= 10:
            success = False
            print("speed invalid")
        if goal.approach_distance <= 0.0 or goal.approach_distance >= 10:
            success = False
            print("approach_distance invalid")

        if not success:
            self.actionserver.set_aborted()
            return

        go_time = time.time() + 90 #timer

        while time.time() < go_time:
            #case1: if front there is distance all around
            if self.front_distance > goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance > goal.approach_distance:
                self.robot_controller.set_move_cmd(0.23, 0.75)
                self.robot_controller.publish()
                print("look left")
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.23, -0.75)
                print("look right")
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)

            #case2: if there is no distance in front
            elif self.front_distance < goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance > goal.approach_distance: 
                if self.left_distance > self.right_distance:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0, 1)
                elif self.left_distance < self.right_distance:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0, -1)#
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            
            #case3: if there is no distance around left or right
            elif self.front_distance > goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance < goal.approach_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.25, 0)#
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            
            #case4: if there is no distance on the right
            elif self.front_distance > goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance < goal.approach_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.3, 1)
                self.robot_controller.publish()
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.3, -0.25)
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            
            #case5: if there is no distance on the left
            elif self.front_distance > goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance > goal.approach_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.3, -1)
                self.robot_controller.publish()
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.3, 0.25)
                self.robot_controller.publish()
           
           #case6: if there is no distance on the left and front
            elif self.front_distance < goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance > goal.approach_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0, -0.25)
                self.robot_controller.publish()
            
            #case7: if there is no distance on the right and front
            elif self.front_distance < goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance < goal.approach_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0, 0.5)
                self.robot_controller.publish()
           
           #case8: if there is no distance anywhere
            elif self.front_distance < goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance < goal.approach_distance:
                if self.left_distance > self.right_distance:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0, 2)
                    self.robot_controller.publish()
                elif self.left_distance < self.right_distance:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0, -2)
                    self.robot_controller.publish() 
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
               
        if success:
            rospy.loginfo('Stop sucessfully.')
            self.result.total_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
            self.result.closest_object_distance = self.front_distance
            self.result.closest_object_angle = self.front_angle
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
            
                
if __name__ == '__main__':
    rospy.init_node('search_action_server')
    ActionServer()
    rospy.spin()
