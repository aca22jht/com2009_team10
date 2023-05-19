#!/usr/bin/env python3
import rospy 
import actionlib
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import Image 
import numpy as np

class PathFollow():
    def callback_function(self, topic_data: Odometry):
        if self.startup:
            rospy.loginfo("Setting vars")
            self.pos_x = topic_data.pose.pose.position.x
            self.pos_y = topic_data.pose.pose.position.y
            self.startup = False

    def __init__(self):
        self.node_name = "path_follow"
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(20)

        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

        self.startup = True
        self.pos_x = 0.0
        self.pos_y = 0.0

    def zone_detector(self):
        pos_x = self.pos_x
        pos_y = self.pos_y
        if (-2.154 <= pos_x <= -1.9722229257346162) and (-2.11820860299087 <= pos_y <= -1.8561134249203455): # This is zone A
            return "A"
        if (-1.2499513744568171 <= pos_x <= -1.0316773335616904) and (2.0195871206906557 <= pos_y <= 2.26014292750731):  # This is zone B
            return "B"
        if (2.006087983504334 <= pos_x <= 2.15057375772878) and (2.008324823620115 <= pos_y <= 2.26931502846564):  # This is zone C
            return "C"

    def goal_reached_callback(self, state, result):
        rospy.loginfo("Goal completed with state: %s", result)
        # Check beacon color

    def move_to_marker(self, waypoint):
        rospy.loginfo("Moving to marker")
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for server...")
        client.wait_for_server()  # Wait for the server to start

        rospy.loginfo("Creating goal")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = waypoint[0]
        goal.target_pose.pose.position.y = waypoint[1]
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal")
        client.send_goal(goal, done_cb=self.goal_reached_callback)
        if client.wait_for_result(rospy.Duration.from_sec(30.0)):
            rospy.loginfo("Result received!")
            # Pause for 2 seconds after reaching the goal
            rospy.loginfo("Pausing...")
            rospy.sleep(2)
        else:
            rospy.logerr("No result received from move_base")

    def main_loop(self):
        while self.pos_x == 0.0:
            rospy.loginfo(self.pos_x)
            self.rate.sleep()
        rospy.loginfo(f"Current position: ({self.pos_x}, {self.pos_y})")
        zone = self.zone
