#!/usr/bin/env python3

from pickle import TRUE
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError 

from sensor_msgs.msg import Image 
import numpy as np

from geometry_msgs.msg import Twist

# Initialisations: 
node_name = "object_detection_node"
rospy.init_node(node_name)
print(f"Launched the '{node_name}' node. Currently waiting for an image...")
rate = rospy.Rate(2)

cvbridge_interface = CvBridge() 

dominant_color = None
waiting_for_image = True 
found_color = False

pub= rospy.Publisher("/cmd_vel", Twist, queue_size=10)

twist = Twist()
twist.linear.x = 0
twist.angular.z = 0.1
pub.publish(twist)

def camera_cb(img_data): 
    global waiting_for_image
    global dominant_color
    global found_color

    if found_color: return

    print("camera calback")
    try:
        #get image
        cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        print("cv_img success") 
    except CvBridgeError as e:
        print(e)

    #get hsv img
    hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    colors = {
        'red': [(0, 100, 100), (10, 255, 255)],
        'blue': [(100, 150, 0), (140, 255, 255)],
        'green': [(35, 100, 50), (85, 255, 255)],
        'turquoise': [(85,200,100), (93, 250, 255)],
        'purple': [(145,160,100), (153, 255, 255)],
        'yellow': [(25,200,100), (40, 255, 255)]
    }

    dominant_color = None
    max_count = 0

    for color, (lower, upper) in colors.items():
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(hsv_img, lower, upper)
        count = cv2.countNonZero(mask)
        print(count)
        total_pixel_count = hsv_img.shape[0] * hsv_img.shape[1]
        #if the amount of a color of pixel is dominant in the image, it is in that zone
        if count > total_pixel_count/2:
            print("resetting dominant color")
            max_count = count
            dominant_color = color

    if dominant_color:
        print(f"The dominant color in the initial zone is {dominant_color}")
        found_color = True
    

rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb) 

while not found_color:
    pub.publish(twist)
    rate.sleep()


twist.angular.z = 0
pub.publish(twist)


cv2.destroyAllWindows() 
