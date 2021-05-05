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

class colour_search(object):

    def __init__(self):
        rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)
        self.rate = rospy.Rate(5)
        self.m00 = 0
        self.m00_min = 10000
        self.turn = False
        self.color_name = ""
        self.lower_bound = []
        self.upper_bound = []
        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.hsv_img = np.zeros((1080,1920,3), np.uint8)
        self.find_target = False

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

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def get_init_color(self):
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
                self.color_name = color_name
                self.lower_bound = lower_bound
                self.upper_bound = upper_bound
                print("SEARCH INITIATED: The target colour is {}".format (self.color_name))
                break
    
    def go_foward(self):
        self.robot_controller.set_move_cmd(0.2, 0)    
        self.robot_controller.publish()
        time.sleep(5)
        self.robot_controller.stop()

    def rotate(self, degree, speed):
        time.sleep(1)
        time_cal = math.radians(degree) / speed
        self.robot_controller.set_move_cmd(0.0, speed)    
        self.robot_controller.publish()
        time_cal = abs(time_cal)
        time.sleep(time_cal)
        self.robot_controller.stop()

    def main(self):
        while not self.ctrl_c:
            if self.turn == False:
                self.rotate(100, 0.2)
                self.get_init_color()
                self.rotate(100, -0.2)
                self.turn = True
            else:
                if self.m00 > self.m00_min and self.find_target == False:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                            print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                            self.find_target = True
                    else: 
                        self.move_rate = 'slow'
                elif self.find_target == True:
                    self.robot_controller.stop()
                    break
                else:
                    self.move_rate = 'fast'
                    
                if self.find_target == False:    
                    if self.move_rate == 'fast':
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                    elif self.move_rate == 'slow':
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                    elif self.move_rate == 'stop':
                        self.robot_controller.set_move_cmd(0.0, 0.0)
                    else:
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                    
                    self.robot_controller.publish()
                    self.rate.sleep()
            
if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
