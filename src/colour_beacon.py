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

class colour_beacon(object):

    def __init__(self):
        rospy.init_node('color_beacon')
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
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
        self.rate = rospy.Rate(5)
        self.m00 = 0
        self.m00_min = 1000000
        self.turn = False
        self.color_name = ""
        self.lower_bound = []
        self.upper_bound = []
        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.hsv_img = np.zeros((1080,1920,3), np.uint8)
        self.find_target = False
        self.stop_at_target = False
        self.raw_data = np.array(tuple())

        # define a Twist instance, which can be used to set robot velocities
        self.all_distance = 0.0
        self.all_angle = 0.0
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

        color_forwards = ""
    
    def scan_callback_function(self, scan_data):


        # all detection
        all_left_arc = scan_data.ranges[0:91]
        all_right_arc = scan_data.ranges[-90:]
        all_arc = np.array(all_left_arc[::-1] + all_right_arc[::-1])
        all_arc_angle = np.arange(-90, 91)

        # find the miniumum object distance within the frontal laserscan arc:
        self.all_distance = all_arc.min()
        self.all_angle = all_arc_angle[np.argmin(all_arc)]

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
        right_arc = scan_data.ranges[315:351]
        right_side_arc = np.array(right_arc[::1])
        right_arc_angle = np.arange(315,351)

        # find the miniumum object distance within the right laserscan arc:
        self.right_distance = right_side_arc.min()
        self.right_angle = right_arc_angle[np.argmin(right_side_arc)]

        #left detection
        left_arc = scan_data.ranges[17:53]
        left_side_arc = np.array(left_arc[::1])
        left_arc_angle = np.arange(17,53)

        # find the miniumum object distance within the left laserscan arc:
        self.left_distance = left_side_arc.min()
        self.left_angle = left_arc_angle[np.argmin(left_side_arc)]

        self.raw_data = np.array(scan_data.ranges)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.hsv_img = hsv_img

        if len(self.lower_bound) > 0 and len(self.upper_bound) > 0:
            self.mask = cv2.inRange(hsv_img, self.lower_bound, self.upper_bound)        
        
        m = cv2.moments(self.mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)
        self.cz = m['m01'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), int(self.cz)), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)



    def get_init_color(self):
        color_threshold = {
            "Red":    ([0, 185, 100], [10, 255, 255]),
            "Blue":   ([115, 224, 100],   [130, 255, 255]),
            "Green":   ([25, 150, 100], [70, 255, 255]),
            "Turquoise":   ([75, 150, 100], [100, 255, 255]),
            "Yellow": ([28, 180, 100], [32, 255, 255]),
            "Purple":   ([145, 190, 100], [155, 255, 255])
        }

        for color_name, (lower, upper) in color_threshold.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            
            if mask.any():
                self.color_name = color_name
                self.lower_bound = lower_bound
                self.upper_bound = upper_bound
                print("SEARCH INITIATED: The target beacon colour is {}.".format (self.color_name))
                break

    def get_color(self):
        color_threshold = {
            "Red":    ([0, 185, 100], [10, 255, 255]),
            "Blue":   ([115, 224, 100],   [130, 255, 255]),
            "Green":   ([25, 150, 100], [70, 255, 255]),
            "Turquoise":   ([75, 150, 100], [100, 255, 255]),
            "Yellow": ([28, 180, 100], [32, 255, 255]),
            "Purple":   ([145, 185, 100], [150, 250, 255])
        }

        for color_name, (lower, upper) in color_threshold.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            if mask.any():
                print("The  colour in front is {}".format (color_name))
                return color_name
                break
    
    #robot should look left and right as it moves forwards
    def move_around(self, distance):
        toggle_waddle = False
        
        if self.front_distance > distance and self.left_distance > distance and self.right_distance > distance:
            
            if toggle_waddle:
                self.robot_controller.set_move_cmd(0.25, 0.3)
                self.robot_controller.publish()
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.25, 0.3)
                self.robot_controller.publish()
                toggle_waddle = False
            else:
                self.robot_controller.set_move_cmd(0.25, 0)
                self.robot_controller.publish()
                toggle_waddle = True
        #case2: if there is no distance in front
        elif self.front_distance < distance and self.left_distance > distance and self.right_distance > distance: 
            if self.left_distance > self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(-0.2, 0.4)
            elif self.left_distance < self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(-0.2, -0.4)#
            self.robot_controller.publish()
        #case3: if there is no distance around left or right
        elif self.front_distance > distance and self.left_distance < distance and self.right_distance < distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0.25, 0)#
            self.robot_controller.publish()
        #case4: if there is no distance on the right
        elif self.front_distance > distance and self.left_distance > distance and self.right_distance < distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0.25, 0.4)
            self.robot_controller.publish()
        #case5: if there is no distance on the left
        elif self.front_distance > distance and self.left_distance < distance and self.right_distance > distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0.25, -0.4)
            self.robot_controller.publish()
        #case6: if there is no distance on the left and front
        elif self.front_distance < distance and self.left_distance < distance and self.right_distance > distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0, -0.4)
            self.robot_controller.publish()
        #case7: if there is no distance on the right and front
        elif self.front_distance < distance and self.left_distance > distance and self.right_distance < distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0, 0.4)
            self.robot_controller.publish()
        #case8: if there is no distance anywhere
        elif self.front_distance < distance and self.left_distance < distance and self.right_distance < distance:
            if self.left_distance > self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0, 2)
                self.robot_controller.publish()
            elif self.left_distance < self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0, -2)
                self.robot_controller.publish() 

    def avoid_to_target(self, distance):
        if self.front_distance > 0.4:
            if self.right_distance > 0.4 and self.left_distance < 0.4:
                self.robot_controller.set_move_cmd(linear=0.05, angular=-0.6)
            elif self.right_distance < 0.4 and self.left_distance > 0.4:
                self.robot_controller.set_move_cmd(linear=0.05, angular=0.6)
            elif self.right_distance < 0.4 and self.left_distance < 0.4:
                self.robot_controller.set_move_cmd(linear=0.05, angular=-0.6)
            else:
                self.robot_controller.set_move_cmd(linear=0.12, angular=0)
                if self.front_distance > 0.9:
                    self.robot_controller.set_move_cmd(linear=0.26, angular=0)
        else:
            if self.right_distance > 0.4 and self.left_distance > 0.4:
                self.robot_controller.set_move_cmd(linear=0.0, angular=-0.8)
            elif self.right_distance > 0.4 and self.left_distance < 0.4:
                self.robot_controller.set_move_cmd(linear=0.0, angular=-0.6)
            elif self.right_distance < 0.4 and self.left_distance > 0.4:
                self.robot_controller.set_move_cmd(linear=0.0, angular=0.6)
            else:
                self.robot_controller.set_move_cmd(linear=0.0, angular=-0.6)
        self.robot_controller.publish()


    def beacon(self, distance):
        #print(self.stop_at_target)
        if self.stop_at_target == False:
            if self.cy >= 560-100 and self.cy <= 560+100:
                color_forwards = self.get_color()
                # self.move_around(0.25)
                self.robot_controller.set_move_cmd(0.15, 0)
                if self.all_distance <= 0.24:
                    if self.right_distance <= 0.24:
                        self.robot_controller.set_move_cmd(-0.25, 0.1)
                        try_left = False
                    elif self.left_distance <= 0.24:
                        self.robot_controller.set_move_cmd(-0.25, -0.1)
                        try_left = True
                    elif self.front_distance <= 0.24:
                        self.robot_controller.set_move_cmd(-0.2, 0)
                    
                    self.robot_controller.publish()
                    

                    print("Too close to wall!!")

                elif (self.front_distance <= 0.3 or self.right_distance <= 0.3 or self.left_distance <= 0.3) and color_forwards == self.color_name:
                    self.robot_controller.stop()
                    self.move_rate = "stop"
                    self.find_target = True
                    if self.front_distance < distance:
                        print("BEACONING COMPLETE: The robot has now stopped.")
                        self.robot_controller.stop()
                        self.stop_at_target = True
                        
                    else:
                        #self.robot_controller.set_move_cmd(0.2, 0)
                        #self.robot_controller.publish()
                        self.move_around(0.5)

            elif 0 < self.cy and self.cy <= 560-100:
                self.robot_controller.set_move_cmd(0.1, 0.1)
                self.robot_controller.publish()
                #print("left")
            elif self.cy > 560+100:
                self.robot_controller.set_move_cmd(0.1, -0.1)
                self.robot_controller.publish()
                #print("right")
            else:
                self.move_around(0.5)
                #print("else")
        else: self.robot_controller.stop()

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
            if self.turn == False:
                self.rotate(90, -0.6)
                self.get_init_color()
                self.rotate(90, 0.6)
                self.robot_controller.stop()
                rospy.sleep(1)
                self.go_foward()
                self.turn = True
                self.init_x = self.robot_odom.posx
                self.init_y = self.robot_odom.posy
                color_forwards = self.get_color()

            else:
                if self.m00 > self.m00_min and self.find_target == False:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            if int(self.cz) > 180 and int(self.cz) < 220:
                                self.move_rate = 'stop'  
                                self.find_target = True
                                print("BEACON DETECTED: Beaconing initiated.")
                    else: 
                        self.move_rate = 'slow'
                elif self.find_target == True and self.stop_at_target == False:
                    self.move_rate = 'beacon'
                elif self.stop_at_target == True:
                    self.move_rate = 'done'
                else:
                    self.move_rate = 'fast'
                      
                if self.move_rate == 'fast':
                    self.move_around(0.5)
                elif self.move_rate == 'slow':
                    if self.cy <= 560-100:
                        self.robot_controller.set_move_cmd(0.1, 0.2)
                        self.robot_controller.publish()
                        #print("slow left")
                    elif self.cy > 560+100:
                        self.robot_controller.set_move_cmd(0.1, -0.2)
                        self.robot_controller.publish()
                        #print("slow right")
                elif self.move_rate == 'stop':
                    self.robot_controller.stop()
                elif self.move_rate == 'beacon':
                    self.beacon(0.35)
                elif self.move_rate == 'done':
                    #self.robot_controller.set_move_cmd(0.1, 0)
                    #self.robot_controller.publish()
                    #rospy.sleep(2)
                    break
                
                self.robot_controller.publish()
                self.rate.sleep()
            
if __name__ == '__main__':
    colour_beacon = colour_beacon()
    try:
        colour_beacon.main()
    except rospy.ROSInterruptException:
        pass
