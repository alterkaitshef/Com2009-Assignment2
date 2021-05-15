#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
import math
from math import sqrt, pow, pi
import numpy as np
import time
from sensor_msgs.msg import LaserScan
from math import sqrt, degrees
import sys

class maze_navigation(object):

    def __init__(self):
        rospy.init_node('turn_and_face')
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.scanSub = rospy.Subscriber('scan', LaserScan, self.scan_callback_function)
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)
        self.rate = rospy.Rate(100)
        self.m00 = 0
        self.m00_min = 1000000
        self.start = False
        self.color_name = ""
        self.lower_bound = []
        self.upper_bound = []
        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.hsv_img = np.zeros((1080,1920,3), np.uint8)
        self.find_target = False
        self.stop_at_target = False
        self.raw_data = np.array(tuple())

        # define a Twist instance, which can be used to set robot velocities
        self.front_distance = 0.0
        self.front_angle = 0.0
        self.right_distance = 0.0
        self.right_angle = 0.0
        self.left_distance = 0.0
        self.left_angle = 0.0
        self.back_distance = 0.0
        self.back_angle = 0.0
        self.init_x = 0.0
        self.init_y = 0.0
        self.small_front_distance = 0
        self.small_front_angle = 0
        self.distance = 0.5
    
    def scan_callback_function(self, scan_data):

        # front detection
        front_left_arc = scan_data.ranges[0:16]
        front_right_arc = scan_data.ranges[-15:]
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angle = np.arange(-15, 16)

        # find the miniumum object distance within the frontal laserscan arc:
        self.front_distance = front_arc.min()
        self.front_angle = front_arc_angle[np.argmin(front_arc)]

        #small front 
        small_front_left_arc = scan_data.ranges[0:6]
        small_front_right_arc = scan_data.ranges[-5:]
        small_front_arc = np.array(small_front_left_arc[::-1] + small_front_right_arc[::-1])
        small_front_arc_angle = np.arange(-5, 6)

        # find the miniumum object distance within the frontal laserscan arc:
        self.small_front_distance = small_front_arc.min()
        self.small_front_angle = small_front_arc_angle[np.argmin(small_front_arc)]
        
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
        self.raw_data = np.array(scan_data.ranges)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def rotate(self, degree, speed):
        rospy.sleep(1)
        time_cal = math.radians(degree) / speed
        self.robot_controller.set_move_cmd(0.0, speed)    
        self.robot_controller.publish()
        time_cal = abs(time_cal)
        rospy.sleep(time_cal)
        self.robot_controller.stop()

    def go_foward(self):
        self.robot_controller.set_move_cmd(0.2, 0)    
        self.robot_controller.publish()
        rospy.sleep(1)
        self.robot_controller.stop()

    def main(self):
        while not self.ctrl_c:
            if self.start == False:
                self.go_foward()
                self.start = True
            else:
                #if self.front_distance > self.distance and self.left_distance > self.distance and self.right_distance > self.distance:
                    #self.robot_controller.set_move_cmd(0.2, 0)
                    #self.robot_controller.publish()
                #case2: if there is no distance in front
                #elif self.front_distance < self.distance and self.left_distance > self.distance and self.right_distance > self.distance: 
                    #if self.left_distance > self.right_distance:
                        #self.robot_controller.stop()
                        #time.sleep(2) 
                        #self.rotate(90, 0.6)
                    #elif self.left_distance < self.right_distance:
                        #self.robot_controller.stop()
                        #time.sleep(2) 
                        #self.rotate(90, -0.6)
                    #self.robot_controller.publish()
                #case3: if there is no distance around left or right
                #elif self.front_distance > self.distance and self.left_distance < self.distance and self.right_distance < self.distance:
                    #self.robot_controller.stop()
                    #time.sleep(2) 
                    #self.robot_controller.set_move_cmd(0.25, 0)#
                    #self.robot_controller.publish()
                #case4: if there is no distance on the right
                #elif self.front_distance > self.distance and self.left_distance > self.distance and self.right_distance < self.distance:
                    #self.robot_controller.stop()
                    #time.sleep(2) 
                    #self.rotate(90, 0.6)
                #case5: if there is no distance on the left
                #elif self.front_distance > self.distance and self.left_distance < self.distance and self.right_distance > self.distance:
                    #self.robot_controller.stop()
                    #time.sleep(2) 
                    #self.rotate(90, -0.6)
                #case6: if there is no distance on the left and front
                if self.front_distance < self.distance and self.left_distance < self.distance:
                    self.robot_controller.stop()
                    time.sleep(2)
                    self.robot_controller.set_move_cmd(0, -0.6)
                    self.robot_controller.publish() 
                    #self.rotate(90, -0.6)
                #case7: if there is no distance on the right and front
                elif self.front_distance < self.distance and self.right_distance < self.distance:
                    self.robot_controller.stop()
                    time.sleep(2) 
                    #self.rotate(90, 0.6)
                    self.robot_controller.set_move_cmd(0, 0.6)
                    self.robot_controller.publish() 
                else:
                    self.robot_controller.set_move_cmd(0.2, 0)
                    self.robot_controller.publish()
                
                
        self.robot_controller.publish()
        self.rate.sleep()
            
if __name__ == '__main__':
    maze_ob = maze_navigation()
    try:
        maze_ob.main()
    except rospy.ROSInterruptException:
        pass
