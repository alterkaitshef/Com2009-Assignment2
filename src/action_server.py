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

        #self.twistPub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scanSub = rospy.Subscriber('scan', LaserScan, self.callback_function)
        #self.odomSub = rospy.Subscriber('odom', Odometry, self.callback_function3)

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        # define the robot pose variables and set them all to zero to start with:
        self.x0 = 0.0
        self.y0 = 0.0
        self.stop_distance = 0
        self.desired_velocity = 0
        
        # define a Twist instance, which can be used to set robot velocities
        self.distance = 0.0
        self.angle = 0.0

    def callback_function(self, scan_data):
        # LaserScan stuff
        left_arc = scan_data.ranges[0:21]

        right_arc = scan_data.ranges[-20:] # last 10 elements in array

        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        arc_angle = np.arange(-20,21)

        # find the miniumum object distance within the frontal laserscan arc:
        self.distance = front_arc.min()
        self.angle = arc_angle[np.argmin(front_arc)]#np.where(front_arc == np.amin(front_arc))[0][0]*scan_data.angle_increment*(180/math.pi)
        #print(self.angle)
        #print(scan_data.angle_increment)
    
    def action_server_launcher(self, goal):
        success = True
        x = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))

        # check goal
        if goal.fwd_velocity <= 0 or goal.fwd_velocity >= 0.26:
            success = False
            print("speed invalid")
        if goal.approach_distance <= 0.2 or goal.approach_distance >= 3.5:
            success = False
            print("approach_distance invalid")

        if not success:
            self.actionserver.set_aborted()
            return

        while self.distance - x >= goal.approach_distance:
            self.robot_controller.publish()
            if self.actionserver.is_preempt_requested():
                rospy.loginfo('Cancelling moving.')
                self.actionserver.set_preempted()
                # stop the robot:
                self.robot_controller.stop()
                success = False
                # exit the loop:
                break
            
            self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)
            #self.rate.sleep()
            self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
            self.actionserver.publish_feedback(self.feedback)
            
            

        if success:
            rospy.loginfo('Stop sucessfully.')
            self.result.total_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
            self.result.closest_object_distance = self.distance
            self.result.closest_object_angle = self.angle
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
            
                
if __name__ == '__main__':
    rospy.init_node('search_action_server')
    SearchSweepAS()
    rospy.spin()
