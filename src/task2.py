#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist

class Obstacle():
   
    def __init__(self):
        global obstacle
        obstacle = Twist()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)  
        self.sub = rospy.Subscriber("/odom", Odometry, self.odometry) 

    def callback(self, msg):
        print ('-------RECEIVING LIDAR SENSOR DATA-------')
        # lidar data @front
        print ('Front: {}'.format(msg.ranges[0]))
        # lidar data @left
        print ('Left:  {}'.format(msg.ranges[90]))
        #lidar data @right
        print ('Right: {}'.format(msg.ranges[270]))
        # lidar data @back
        print ('Back: {}'.format(msg.ranges[180]))
        
      
    # Obstacle Avoidance
        self.distance = 1.0
        if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance:

        # if theres no obstacle detected :
            obstacle.linear.x = 0.5 
            obstacle.angular.z = 0.1 
            rospy.loginfo("Exploring")

        # incoming obstacle :
        else: 
            if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance:
                rospy.loginfo("An Obstacle Near Detected")
                # stop
                obstacle.linear.x = 0.0
                # rotate counter-clockwise
                obstacle.angular.z = 0.5

            else:
                obstacle.linear.x = 0.0
                # rotate clockwise
                obstacle.angular.z = -0.5

            # distance to the rightmost laser scan measurement, range value at an angle of 15 degrees, 345 degrees, 45 degrees and 315 degrees are > certain distance threshold? No obstacles
            if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:               
                obstacle.linear.x = 0.5 
                obstacle.angular.z = 0.1

        self.pub.publish(obstacle)

    def odometry(self, msg):
    # position and orientation of turtlebot
        print (msg.pose.pose)

if __name__ == '__main__':
    # initializing the node
    rospy.init_node('obstacle_avoidance_node')
    Obstacle()
    # looping it
    rospy.spin()
