#!/usr/bin/env python3

from multiprocessing.context import ForkContext
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class task5:

    def __init__(self):
        self.node_name = "task5"

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.lidar_subscriber = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        # Initialize the node
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(15)  

        self.min_distance_threshold = 0.45
        self.front_arc = np.empty(180)
        self.robot_command = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.velocity_publisher.publish(Twist())
        self.ctrl_c = True

    def solve_maze(self):
        """
        Main loop for the maze-solving algorithm.
        """
        while not self.ctrl_c:
            front_distance = np.amin(self.front_arc[75:105])
            right_side_distance = np.amin(self.front_arc[120:150])

            if front_distance > self.min_distance_threshold:
                if right_side_distance < 0.3:
                    print("Turning left")
                    self.robot_command.angular.z = 0.9 
                    self.robot_command.linear.x = 0.26 
                elif 0.3 < right_side_distance < 0.4:
                    print("Going straight")
                    self.robot_command.angular.z = 0 
                    self.robot_command.linear.x = 0.26 
                else:
                    print("Turning right")
                    self.robot_command.angular.z = -0.9
                    self.robot_command.linear.x = 0.26 
            else:
                print("Front obstacle detected. Turning away.")
                self.robot_command.angular.z = 0 
                self.robot_command.linear.x = 0 
                self.velocity_publisher.publish(self.robot_command)
                self.robot_command.angular.z = 1.2
                self.velocity_publisher.publish(self.robot_command)

            self.velocity_publisher.publish(self.robot_command)
            self.rate.sleep()

    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:91]
        right_arc = scan_data.ranges[-90:]
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = self.front_arc.min()

        arc_angles = np.arange(-90, 91)
        self.object_angle = arc_angles[np.argmin(self.front_arc)]


if __name__ == '__main__':
    maze_solver_instance = task3()
    try:
        maze_solver_instance.solve_maze()
    except rospy.ROSInterruptException:
        pass

    except rospy.ROSInterruptException:
        pass
