#!/usr/bin/env python3

from multiprocessing.context import ForkContext
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class Task3Maze:

    def __init__(self):
        self.node_name = "task3"

        self.pub_velocity = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub_lidar = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        #iniate the node
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(15)  

        self.min_distance = 1
        self.front_arc = np.empty(180)
        self.cmd = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        #rospy.loginfo(f"the {self.node_name} node has been initialised...")

    def shutdownhook(self):
        self.pub_velocity.publish(Twist())
        self.ctrl_c = True

    def main_loop(self):

        while not self.ctrl_c:

            front = np.amin(self.front_arc[75:105])
            right_Side = np.amin(self.front_arc[120:150])

            if front > 0.45:
                if right_Side < 0.3:
                    print("turn left")
                    self.cmd.angular.z = 0.9 
                    self.cmd.linear.x = 0.26 
                elif 0.3 < right_Side < 0.4:
                    print("go straight")
                    self.cmd.angular.z = 0 
                    self.cmd.linear.x = 0.26 
                else:
                    print("turn right")
                    self.cmd.angular.z = -0.9
                    self.cmd.linear.x = 0.26 
            else:
                print("Front obstacle detected. Turning away.")
                self.cmd.angular.z = 0 
                self.cmd.linear.x = 0 
                self.pub_velocity.publish(self.cmd)
                self.cmd.angular.z = 1.2
                self.pub_velocity.publish(self.cmd)

            self.pub_velocity.publish(self.cmd)
            self.rate.sleep()

    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:91]
        right_arc = scan_data.ranges[-90:]
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = self.front_arc.min()

        arc_angles = np.arange(-90, 91)
        self.object_angle = arc_angles[np.argmin(self.front_arc)]

if __name__ == '__main__':
    task3Maze_instance = Task3Maze()
    try:
        task3Maze_instance.main_loop()
    except rospy.ROSInterruptException:
        pass    
