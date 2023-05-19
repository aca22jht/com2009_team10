#!/usr/bin/env python3
import rospy 
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from nav_msgs.msg import Odometry
from pickle import TRUE
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError 

from sensor_msgs.msg import Image 
import numpy as np

from geometry_msgs.msg import Twist



class PathFollow(): 

    #gets the current position of the robot 
    def callback_function(self, topic_data: Odometry):
        if self.startup:
            print("setting vars")
            self.pos_x = topic_data.pose.pose.position.x
            self.pos_y = topic_data.pose.pose.position.y
            self.startup = False
            self.cvbridge_interface = CvBridge() 

    def camera_cb(self, img_data): 
        try:
            #get image
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
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

        if self.searching:
            #do the stuff to see what color the beacon is
            rospy.loginfo(f"We are searching for a color beacon...") 
            max_count = 0
            for color, (lower, upper) in colors.items():
                lower = np.array(lower, dtype=np.uint8)
                upper = np.array(upper, dtype=np.uint8)
                mask = cv2.inRange(hsv_img, lower, upper)
                count = cv2.countNonZero(mask)
                total_pixel_count = hsv_img.shape[0] * hsv_img.shape[1]
                #if the amount of a color of pixel is dominant in the image, it is in that zone
                if count > max_count:
                    beacon_color = color
                    max_count = count
            rospy.loginfo(f"Detected beacon is {beacon_color}")
            self.beacon_determined = beacon_color
        elif not self.found_color:
            for color, (lower, upper) in colors.items():
                lower = np.array(lower, dtype=np.uint8)
                upper = np.array(upper, dtype=np.uint8)
                mask = cv2.inRange(hsv_img, lower, upper)
                count = cv2.countNonZero(mask)
                total_pixel_count = hsv_img.shape[0] * hsv_img.shape[1]
                #if the amount of a color of pixel is dominant in the image, it is in that zone
                if count > total_pixel_count/2:
                    print(f"The dominant color in the initial zone is {self.dominant_color}")
                    self.found_color = True
                    self.dominant_color = color
                

    def __init__(self): 
        self.node_name = "path_follow" 
        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(20) 

        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

        self.startup = True
        self.pos_x = 0.0
        self.pos_y = 0.0

        print(f"Launched the '{self.node_name}' node. Currently waiting for an image...")
        
        self. dominant_color = None
        self.found_color = False
        self.searching = False
        self.beacon_determined = None

        self.pub= rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.twist = Twist()
        self.twist.linear.x = 0
        self. twist.angular.z = 0.0

    def zone_detector(self):
        pos_x = self.pos_x
        pos_y = self.pos_y
        if (-2.154 <= pos_x <= -1.9722229257346162) and (-2.11820860299087 <= pos_y <= -1.8561134249203455): # This is zone A
            return "A"
        if (-1.2499513744568171 <= pos_x <= -1.0316773335616904) and (2.0195871206906557 <= pos_y <= 2.26014292750731):  # This is zone B
            return "B"
        if (2.006087983504334 <= pos_x <= 2.15057375772878) and (2.008324823620115 <= pos_y <= 2.26931502846564):  # This is zone C
            print("is in zone c")
            return "C"

    def check_beacon(self):
        while self.beacon_determined == None:
            self.searching = True
        print("beacon determined")
        self.beacon_determined = None
        self.searching = False

    def goal_reached_callback(self, state, result ):
        rospy.loginfo("goal completed with state: %s", result)
        #check beacon
        self.check_beacon()
        

    def move_to_marker(self, waypoint):
        rospy.loginfo("moving to marker")
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for server...")
        client.wait_for_server()  # Wait for the server to start

        rospy.loginfo("Creating goal")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        rospy.loginfo("Waypoint coordinates: %s, %s", waypoint[0], waypoint[1])
        goal.target_pose.pose.position.x = waypoint[0][0]
        goal.target_pose.pose.position.y = waypoint[0][1]
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypoint[1][2]
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal")
        client.send_goal(goal, done_cb=self.goal_reached_callback)
        if client.wait_for_result(rospy.Duration.from_sec(30.0)):
            rospy.loginfo("Result received!")
            rospy.loginfo("Pausing to check beacon")
            #here, spin to check beacon
            self.check_beacon()
            print("beacon has been checked")
        else:
            rospy.logerr("No result received from move_base")

    def main_loop(self):
        while self.pos_x == 0.0:
            print(self.pos_x)
            self.rate.sleep()
        
        while not self.found_color:
            self.twist.angular.z = 0.1
            self.pub.publish(self.twist)
            self.rate.sleep()

        self.twist.angular.z = 0
        self.pub.publish(self.twist)

        if self.zone_detector() == "A": # This is zone A path
            print("cooking in here")
            way_points = [((-0.062,-0.226), (0,0,1.644,1.0)) , 
           ( (-1.846,0.039),(0,0,-2.30,1.0)), 
            ((-0.7030,1.415),(0,0,1.02,1.0)),
            ((0.962,1.099), (0,0,-2.20,1.0)),
            ((1.835,-1.242),(0,0, -1.60 ,1.0))]
            for waypoint in (way_points):
                self.move_to_marker(waypoint)
        elif  self.pos_x == -1.24044 and self.pos_y == 2.06729:  # This is zone B path
            way_points = [(-0.774, -0.254),(0.916, -1.336),(1.200 ,1.491),(-0.717,1.092)] 
        elif self.pos_x == -2.06729 and  self.pos_y == 1.97396:  # This is zone C path
            way_points = [(1.200 ,1.491),(-0.717,1.092),(-0.774, -0.254),(0.916, -1.336)]
        else:
            rospy.loginfo("The robot is not in a start zone")

    

if __name__ == '__main__': 
    publisher_instance = PathFollow() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass
