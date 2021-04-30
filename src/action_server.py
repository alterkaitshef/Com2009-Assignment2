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
import numpy as np
from sensor_msgs.msg import LaserScan
from math import sqrt

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
        self.front_round_distance = 1.0
        self.back_round_distance = 1.0

        # get start time 
        self.startTime = rospy.get_rostime()
        print(self.startTime.secs)

    def callback_function(self, scan_data):

        all_round_left = scan_data.ranges[0:180]
        all_round_right = scan_data.ranges[-180:]
        all_round_arc = np.array(all_round_right[::-1] + all_round_left[::-1])
        all_round_arc_angle = np.arange(-180, 180)

        self.all_round_distance = all_round_arc.min()


        front_round_left = scan_data.ranges[0:90]
        front_round_right = scan_data.ranges[-90:]
        front_round_arc = np.array(front_round_right[::-1] + front_round_left[::-1])
        front_round_arc_angle = np.arange(-90, 90)

        self.front_round_distance = front_round_arc.min()
        # print("all_round distance: {}".format(self.all_round_distance))

        back_round_arc = np.array(front_round_right[::1] + front_round_left[::1])
        self.back_round_distance = np.arange(90,-90)

        self


        # front detection
        front_left_arc = scan_data.ranges[0:16]
        front_right_arc = scan_data.ranges[-15:]
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angle = np.arange(-15, 16)

        # find the miniumum object distance within the frontal laserscan arc:
        self.front_distance = front_arc.min()
        self.front_angle = front_arc_angle[np.argmin(front_arc)]
        # print("front distance: {}".format(self.front_distance))
        #print("front angle: {}".format(self.front_angle))
        
        #right detection
        right_arc = scan_data.ranges[314:344]
        right_side_arc = np.array(right_arc[::1])
        right_arc_angle = np.arange(314,344)

        # find the miniumum object distance within the right laserscan arc:
        self.right_distance = right_side_arc.min()
        self.right_angle = right_arc_angle[np.argmin(right_side_arc)]
        # print("right distance: {}".format(self.right_distance))
        #print("right angle: {}".format(self.right_angle))

        #left detection
        left_arc = scan_data.ranges[16:46]
        left_side_arc = np.array(left_arc[::1])
        left_arc_angle = np.arange(16,46)

        # find the miniumum object distance within the left laserscan arc:
        self.left_distance = left_side_arc.min()
        self.left_angle = left_arc_angle[np.argmin(left_side_arc)]
        # print("left distance: {}".format(self.left_distance))
        #print("left angle: {}".format(self.left_angle))

        #back detection
        left_arc = scan_data.ranges[22:61]
        left_side_arc = np.array(left_arc[::1])
        left_arc_angle = np.arange(22,61)

        # find the miniumum object distance within the left laserscan arc:
        self.left_distance = left_side_arc.min()
        self.left_angle = left_arc_angle[np.argmin(left_side_arc)]
        #print("left distance: {}".format(self.left_distance))
        #print("left angle: {}".format(self.left_angle))

    def robot_stop(self):
        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < 0.25:
            self.robot_controller.set_move_cmd(0, 0)
            self.robot_controller.publish()
    
    def robot_back(self):
        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < 0.01:
            self.robot_controller.set_move_cmd(-0.5, 0)
            self.robot_controller.publish()
    
    def robot_foward(self):
        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < 0.5:
            self.robot_controller.set_move_cmd(0.1, 0)
            self.robot_controller.publish()
    
    def turn_left(self):
        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < 2:
            self.robot_controller.set_move_cmd(0, 0.26)
            self.robot_controller.publish()
    
    def turn_right(self):
        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < 2:
            self.robot_controller.set_move_cmd(0, -0.26)
            self.robot_controller.publish()

    
    def action_server_launcher(self, goal):
        success = True
        x = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))

        # check goal
        if goal.fwd_velocity <= 0 or goal.fwd_velocity >= 100:
            success = False
            print("speed invalid")
        if goal.approach_distance <= 0.0 or goal.approach_distance >= 100:
            success = False
            print("approach_distance invalid")

        if not success:
            self.actionserver.set_aborted()
            return

        d = 0.05
        go = True
        while go:
            if self.front_round_distance <= d:
                print("front hit")
                rospy.loginfo('Cancelling the camera sweep.')
                self.actionserver.set_preempted()
                # stop the robot:
                self.robot_controller.stop()
                success = False
                # exit the loop:
                break
            elif self.all_round_distance <= d:
                print("back hit")
                rospy.loginfo('Cancelling the camera sweep.')
                self.actionserver.set_preempted()
                # stop the robot:
                self.robot_controller.stop()
                success = False
                # exit the loop:
                break
            else:
                '''if self.front_distance > goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance > goal.approach_distance:
                    self.robot_controller.set_move_cmd(0.5, 0)#
                    self.robot_controller.publish()
                    self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                    self.actionserver.publish_feedback(self.feedback)
                    #print("----------------------------------------------------front")
                 elif self.front_distance > goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance > goal.approach_distance: 
                    if self.left_distance > self.right_distance:
                        #self.robot_controller.stop()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        self.robot_controller.set_move_cmd(0, 2)#
                        self.robot_controller.publish()'''
                if self.front_distance > goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance > goal.approach_distance:
                    #self.robot_controller.set_move_cmd(0.5, 0)#
                    #self.robot_controller.publish()
                    if self.left_distance > self.right_distance:
                        #self.robot_controller.stop()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        self.robot_controller.set_move_cmd(0.5, 2)#
                        self.robot_controller.publish()
                    elif self.left_distance < self.right_distance:
                        #self.robot_controller.stop()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        self.robot_controller.set_move_cmd(0.5, -2)#
                        self.robot_controller.publish()
                    self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                    self.actionserver.publish_feedback(self.feedback)
                    #print("----------------------------------------------------front")
                elif self.front_distance < goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance > goal.approach_distance: 
                    if self.left_distance > self.right_distance:
                        #self.robot_controller.stop()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        self.robot_controller.set_move_cmd(0, 2)#
                        self.robot_controller.publish()
                    elif self.left_distance < self.right_distance:
                        #self.robot_controller.stop()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        self.robot_controller.set_move_cmd(0, -2)#
                        self.robot_controller.publish()
                    self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                    self.actionserver.publish_feedback(self.feedback)
                    print("----------------------------------------------------decide")
                elif self.front_distance > goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance < goal.approach_distance:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.5, 0)#
                    self.robot_controller.publish()
                    self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                    self.actionserver.publish_feedback(self.feedback)
                elif self.front_distance > goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance < goal.approach_distance:
                    self.robot_controller.stop()
                    #self.robot_controller.set_move_cmd(-2.5, 0)
                    #self.robot_controller.publish()
                    self.robot_controller.set_move_cmd(0.5, 2)#
                    self.robot_controller.publish()
                    self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                    self.actionserver.publish_feedback(self.feedback)
                    print("----------------------------------------------------case3")
                elif self.front_distance > goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance > goal.approach_distance:
                    self.robot_controller.stop()
                    #self.robot_controller.set_move_cmd(-2.5, 0)
                    #self.robot_controller.publish()
                    self.robot_controller.set_move_cmd(0.5, -2)#
                    self.robot_controller.publish()
                    self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                    self.actionserver.publish_feedback(self.feedback)
                    print("----------------------------------------------------case4")
                elif self.front_distance < goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance < goal.approach_distance:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(-5, 0)
                    self.robot_controller.publish()
                    #self.robot_controller.set_move_cmd(-5, 0)
                    #self.robot_controller.publish()
                    self.robot_controller.set_move_cmd(0, 2)#
                    self.robot_controller.publish()
                    self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                    self.actionserver.publish_feedback(self.feedback)
                    print("----------------------------------------------------case5")
                elif self.front_distance < goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance > goal.approach_distance:
                    self.robot_controller.stop()
                    #self.robot_controller.set_move_cmd(-5, 0)
                    #self.robot_controller.publish()
                    #self.robot_controller.set_move_cmd(-5, 0)
                    #self.robot_controller.publish()
                    self.robot_controller.set_move_cmd(0, -2)#
                    self.robot_controller.publish()
                    self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                    self.actionserver.publish_feedback(self.feedback)
                    print("----------------------------------------------------case6")
                elif self.front_distance < goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance < goal.approach_distance:
                    self.robot_controller.stop()
                    if self.left_distance > self.right_distance:
                        self.robot_controller.stop()
                        self.robot_controller.set_move_cmd(-5, 0)
                        self.robot_controller.publish()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        self.robot_controller.set_move_cmd(0, 2)#
                        self.robot_controller.publish()
                    elif self.left_distance < self.right_distance:
                        self.robot_controller.stop()
                        self.robot_controller.set_move_cmd(-5, 0)
                        self.robot_controller.publish()
                        #self.robot_controller.set_move_cmd(-5, 0)
                        #self.robot_controller.publish()
                        self.robot_controller.set_move_cmd(0, -2)#
                        self.robot_controller.publish() 
                    self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                    self.actionserver.publish_feedback(self.feedback)
                    print("----------------------------------------------------case7")
                else:
                    print("----------------------------------------------------edljfkadf")
                

            '''
            if self.right_distance > goal.approach_distance and self.right_distance < self.left_distance:
                difference = (self.right_distance - goal.approach_distance)
                self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.26)
                self.robot_controller.publish()
                
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            elif self.right_distance > goal.approach_distance and self.right_distance > self.left_distance:
                difference = (self.right_distance - goal.approach_distance)
                self.robot_controller.set_move_cmd(goal.fwd_velocity, -0.26)
                self.robot_controller.publish()
                
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            elif self.left_distance > goal.approach_distance and self.right_distance > self.left_distance:
                difference = (self.left_distance - goal.approach_distance)
                self.robot_controller.set_move_cmd(0, -0.26)
                self.robot_controller.publish()
                
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            elif self.left_distance > goal.approach_distance and self.right_distance < self.left_distance:
                difference = (self.left_distance - goal.approach_distance)
                self.robot_controller.set_move_cmd(0, 0.26)
                self.robot_controller.publish()
                
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            else:
                self.robot_controller.set_move_cmd(0, 0.1)
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            '''
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
