#!/usr/bin/env python

import rospy
import os

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

node_name = "object_detection_exercise"
rospy.init_node(node_name)
print("Launched the '{}' node. Currently waiting for an image...".format(node_name))
rate = rospy.Rate(5)

cvbridge_interface = CvBridge()

waiting_for_image = True

def show_and_save_image(img, img_name):

    base_image_path = "/home/student/myrosdata/week6_images"
    full_image_path = os.path.join(base_image_path, "{}.jpg".format(img_name))

    cv2.imshow(img_name, img)
    cv2.waitKey(0)

    cv2.imwrite(full_image_path, img)
    print("Saved an image to '{}'\nimage dims = {}x{}px\nfile size = {} bytes".format(full_image_path, 
            img.shape[0], img.shape[1], os.path.getsize(full_image_path)))

def camera_cb(img_data):
    global waiting_for_image  
    try:
        cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

    if waiting_for_image == True:
        height, width, channels = cv_img.shape

        print("Obtained an image of height {}px and width {}px.".format(height, width))

        show_and_save_image(cv_img, img_name = "step1_original")
    
        '''crop_width = width - 400
        crop_height = 400
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_z0:crop_z0+crop_height, crop_y0:crop_y0+crop_width]'''

        crop_width = width #- 800
        crop_height = 20
        crop_x = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        #cropped_img = cv_img[10:crop_height, 10:crop_x+crop_width]
        cropped_img = cv_img[1060: height, 0:crop_width]


        show_and_save_image(cropped_img, img_name = "step2_cropping")

        '''hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        lower_threshold = (115, 235, 100)
        upper_threshold = (125, 255, 255)
        img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
        show_and_save_image(img_mask, img_name = "step3_image_mask")

        filtered_img = cv2.bitwise_and(cropped_img, cropped_img, mask = img_mask)
        show_and_save_image(filtered_img, img_name = "step4_filtered_image")'''

        waiting_for_image = False

rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)

while waiting_for_image:
    rate.sleep()

cv2.destroyAllWindows()