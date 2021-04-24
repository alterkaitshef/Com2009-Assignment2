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

class SearchSweepAS(object):
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
        self.distance = 0.0
        self.angle = 0.0

        # get start time 
        self.startTime = rospy.get_rostime()
        print(self.startTime.secs)

    def callback_function(self, scan_data):
        # front detection
        front_left_arc = scan_data.ranges[0:1]
        front_right_arc = scan_data.ranges[-1:] # last 10 elements in array
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angle = np.arange(-1,1)

        # find the miniumum object distance within the frontal laserscan arc:
        self.front_distance = front_arc.min()
        self.front_angle = front_arc_angle[np.argmin(front_arc)]
        #print("front distance: {}".format(self.front_distance))
        #print("front angle: {}".format(self.front_angle))
        
        #right detection
        right_arc = scan_data.ranges[250:360]
        right_side_arc = np.array(right_arc[::1])
        right_arc_angle = np.arange(250,360)

        # find the miniumum object distance within the right laserscan arc:
        self.right_distance = right_side_arc.min()
        self.right_angle = right_arc_angle[np.argmin(right_side_arc)]
        #print("right distance: {}".format(self.right_distance))
        #print("right angle: {}".format(self.right_angle))

        #left detection
        left_arc = scan_data.ranges[1:110]
        left_side_arc = np.array(left_arc[::1])
        left_arc_angle = np.arange(1,110)

        # find the miniumum object distance within the left laserscan arc:
        self.left_distance = left_side_arc.min()
        self.left_angle = left_arc_angle[np.argmin(left_side_arc)]
        #print("left distance: {}".format(self.left_distance))
        #print("left angle: {}".format(self.left_angle))

        #back detection 
        back_arc = scan_data.ranges[111:249]
        back_side_arc = np.array(back_arc[::1])
        back_arc_angle = np.arange(111,249)

        # find the miniumum object distance within the back laserscan arc:
        self.back_distance = back_side_arc.min()
        self.back_angle = back_arc_angle[np.argmin(back_side_arc)]
        #print("back distance: {}".format(self.back_distance))
        #print("back angle: {}".format(self.back_angle))

        #all detection 
        all_arc = scan_data.ranges[0:360]
        all_side_arc = np.array(all_arc[::1])
        all_arc_angle = np.arange(0,360)

        # find the miniumum object distance within the back laserscan arc:
        self.all_distance_min = all_side_arc.min()
        self.all_distance_max = all_side_arc.max()
        self.all_angle_min = all_arc_angle[np.argmin(all_side_arc)]
        self.all_angle_max = all_arc_angle[np.argmax(all_side_arc)]
        #print("all distance max: {}".format(self.all_distance_max))
        #print("all angle max: {}".format(self.all_angle_max))
        #print("all distance min: {}".format(self.all_distance_min))
        #print("all angle min: {}".format(self.all_angle_min))
        #print(self.robot_odom.yaw)


    
    def robot_stop(self):
        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < 0.25:
            self.robot_controller.set_move_cmd(0, 0)
            self.robot_controller.publish()
    
    def robot_back(self):
        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < 0.5:
            self.robot_controller.set_move_cmd(-0.1, 0)
            self.robot_controller.publish()
    
    def robot_foward(self):
        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < 0.5:
            self.robot_controller.set_move_cmd(0.1, 0)
            self.robot_controller.publish()
    
    def turn_left(self):
        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < 0.5:
            self.robot_controller.set_move_cmd(0, 0.26)
            self.robot_controller.publish()
    
    def turn_right(self):
        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < 0.5:
            self.robot_controller.set_move_cmd(0, -0.26)
            self.robot_controller.publish()

    
    def action_server_launcher(self, goal):
        success = True
        x = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))

        # check goal
        if goal.fwd_velocity <= 0 or goal.fwd_velocity >= 0.26:
            success = False
            print("speed invalid")
        if goal.approach_distance <= 0.0 or goal.approach_distance >= 100:
            success = False
            print("approach_distance invalid")

        if not success:
            self.actionserver.set_aborted()
            return
        
        while rospy.get_rostime().secs- self.startTime.secs < 60:
            init_right_difference = (self.right_distance - goal.approach_distance)
            init_left_difference = (self.left_distance - goal.approach_distance)
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
                self.robot_controller.set_move_cmd(goal.fwd_velocity, -0.26)
                self.robot_controller.publish()
                
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            elif self.left_distance > goal.approach_distance and self.right_distance < self.left_distance:
                difference = (self.left_distance - goal.approach_distance)
                self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.26)
                self.robot_controller.publish()
                
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            else:
                self.robot_controller.set_move_cmd(0, 0.1)
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            '''elif self.right_distance < goal.approach_distance:
                self.robot_controller.set_move_cmd(0.0, 0.0)
                self.robot_controller.publish()
                self.robot_controller.set_move_cmd(-0.1, 0.26)
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            elif self.left_distance < goal.approach_distance:
                self.robot_controller.set_move_cmd(0.0, 0.0)
                self.robot_controller.publish()
                self.robot_controller.set_move_cmd(-0.2, -0.26)
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            else:
                self.robot_controller.set_move_cmd(0.2, 0)
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)'''

        
            '''
            elif self.back_distance <= 0.3:
                self.robot_stop()
                self.robot_foward()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
                print("back")
            elif self.right_distance <= goal.approach_distance and self.back_distance >= 0.3:
                self.robot_stop()
                self.robot_back()
                self.robot_stop()
                self.turn_left()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
                print("right")
            elif self.left_distance <= goal.approach_distance and self.back_distance >= 0.3:
                self.robot_stop()
                self.robot_back()
                self.robot_stop()
                self.turn_right()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
                print("left")'''
             
        '''  
        while rospy.get_rostime().secs - self.startTime.secs < 60:
            init = self.robot_odom.yaw
            diff_a = abs(self.all_angle_max - init)
            #print (diff_a )

            if diff_a>0:
                #diff_a = abs(self.all_angle_max- self.robot_odom.yaw)
                #print(self.robot_odom.yaw)
                #print ("this is the difference between reference angle and current angle: {}".format (diff_a))
                #print("-------------------------------------------------------------------------------")
                self.robot_controller.set_move_cmd(0.0, 0.1)
                self.robot_controller.publish()
                #self.rate.sleep()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            elif diff_a == 0:
                self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                self.robot_controller.publish()
                #self.rate.sleep()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)'''
                
        if success:
            rospy.loginfo('Stop sucessfully.')
            self.result.total_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
            self.result.closest_object_distance = self.front_distance
            self.result.closest_object_angle = self.front_angle
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()'''
            
                
if __name__ == '__main__':
    rospy.init_node('search_action_server')
    SearchSweepAS()
    rospy.spin()
