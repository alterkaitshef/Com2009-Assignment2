#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import math

# Import all the necessary ROS message types:
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
import numpy as np

class maze_nav(object):

    def __init__(self):
        rospy.init_node('maze_nav')
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        rospy.on_shutdown(self.shutdown_ops)
        self.rate = rospy.Rate(1000)
        
        # variable
        self.ctrl_c = False
        self.check_at_j = 0
        self.left_arc = 0
        self.right_arc = 0
        self.right_status = False
        self.left_status = False
        self.front_status = False

    def shutdown_ops(self):
        self.robot_controller.stop()
        self.ctrl_c = True
    
    def callback(self, scan_data):
        # left split
        left_arc30_40 = scan_data.ranges[30:40]
        left_arc40_50 = scan_data.ranges[40:50]
        left_arc30_50 = np.array(left_arc40_50[::-1] + left_arc30_40[::-1])
        self.left_arc = left_arc30_50.min()

        # right split
        right_arc30_40 = scan_data.ranges[-40:-30]
        right_arc40_50 = scan_data.ranges[-50:-40]
        right_arc30_50 = np.array(right_arc40_50[::-1] + right_arc30_40[::-1])
        self.right_arc = right_arc30_50.min()

        # all
        left_arc = scan_data.ranges[0:40]
        right_arc = scan_data.ranges[-40:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.left = left_arc
        self.right = right_arc 
        self.front_distance = front_arc

        self.left90 = scan_data.ranges[90]
        self.right90 = scan_data.ranges[-90]
        self.front = scan_data.ranges[0]

    def go_foward(self):
        rospy.sleep(1)
        self.robot_controller.set_move_cmd(0.2, 0.0)    
        self.robot_controller.publish()
        rospy.sleep(3)
    
    def turn_right90(self):
        self.robot_controller.set_move_cmd(0.0, -0.62)
        self.robot_controller.publish()
        rospy.sleep(2.5)
        while self.left90 > 0.359:
            self.robot_controller.set_move_cmd(0.0, -0.3)
            self.robot_controller.publish()       
        self.robot_controller.stop()
        rospy.sleep(0.5)
    
    def turn_left90(self):
        self.robot_controller.set_move_cmd(0.0, 0.62)
        self.robot_controller.publish()
        rospy.sleep(2.5)
        while self.right90 > 0.359:
            self.robot_controller.set_move_cmd(0.0, 0.3)
            self.robot_controller.publish()
        self.robot_controller.stop()
        rospy.sleep(0.5)      

    def checkRight(self):
        if self.right90 < 0.5:
            self.right_status = True
        else:
            self.right_status = False
    
    def checkLeft(self):
        if self.left90 < 0.5:
            self.left_status = True
        else:
            self.left_status = False
    
    def checkFront(self):
        if self.front < 0.45:
            self.front_status = True
        else:
            self.front_status = False     
    
    def adjust_bad_position(self):
        if self.left_arc < 0.3:
            self.robot_controller.set_move_cmd(0.0, -0.3)
            self.robot_controller.publish()
        elif self.right_arc < 0.3:
            self.robot_controller.set_move_cmd(0.0, 0.3)
            self.robot_controller.publish()
        else:
            self.robot_controller.set_move_cmd(0.2, 0.0)    
            self.robot_controller.publish()     

    def main(self):
        self.go_foward()
        while not self.ctrl_c:
            # check distance
            self.checkLeft()
            self.checkRight()
            self.checkFront()
            if self.left_status == True and self.front_status == True:
                self.robot_controller.stop()
                rospy.sleep(0.5) 
                self.turn_right90()
            elif self.right_status == True and self.front_status == True:
                self.robot_controller.stop()
                rospy.sleep(0.5) 
                self.turn_left90()
            elif self.front_status == True and self.right_status == False and self.left_status == False:
                if self.check_at_j == 0:
                    self.robot_controller.stop()
                    rospy.sleep(0.5)   
                    self.turn_left90()
                    self.check_at_j += 1
                elif self.check_at_j == 1:
                    self.robot_controller.stop()
                    rospy.sleep(0.5) 
                    self.turn_right90()
                    self.check_at_j += 1
                elif self.check_at_j == 2:
                    self.robot_controller.stop()
                    rospy.sleep(0.5) 
                    self.turn_left90()
            else:
                self.adjust_bad_position()
 
        self.rate.sleep()

if __name__ == '__main__':
    maze_navigation = maze_nav()
    try:
        maze_navigation.main()
    except rospy.ROSInterruptException:
        pass
