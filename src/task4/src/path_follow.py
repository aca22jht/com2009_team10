#!/usr/bin/env python3
import rospy 
import actionlib
import tf2_ros
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from nav_msgs.msg import Odometry


class PathFollow(): 

    def callback_function(self, topic_data: Odometry):
        if self.startup:
            print("setting vars")
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
            print("is in zone c")
            return "C"


    def goal_reached_callback(self, state, result ):
        rospy.loginfo("goal completed with state: %s", result)
        #check beacon
    

    def move_to_marker(self, waypoint):
        rospy.loginfo("moving to marker")
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for server...")
        client.wait_for_server()  # Wait for the server to start

        rospy.loginfo("Creating goal")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        rospy.loginfo("Waypoint coordinates: %s, %s", waypoint[0], waypoint[1])
        goal.target_pose.pose.position.x = waypoint[0]
        goal.target_pose.pose.position.y = waypoint[1]
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal")
        client.send_goal(goal, done_cb=self.goal_reached_callback)
        if client.wait_for_result(rospy.Duration.from_sec(30.0)):
            rospy.loginfo("Result received!")
        else:
            rospy.logerr("No result received from move_base")

    def main_loop(self):
        while self.pos_x == 0.0:
            print(self.pos_x)
            self.rate.sleep()
        print(self.pos_x, self.pos_y)
        print(self.zone_detector())

        if self.zone_detector() == "A": # This is zone A
            print("cooking in here")
            way_points = [(-0.774, -0.254),(-0.717,1.092),(1.200 ,1.491),(0.916, -1.336)]
            for waypoint in (way_points):
                self.move_to_marker(waypoint)
        elif  pos_x == -1.24044 and pos_y == 2.06729:  # This is zone B
            way_points = [(-0.774, -0.254),(0.916, -1.336),(1.200 ,1.491),(-0.717,1.092)] 
        elif pos_x == -2.06729 and  pos_y == 1.97396:  # This is zone C
            way_points = [(1.200 ,1.491),(-0.717,1.092),(-0.774, -0.254),(0.916, -1.336)]
        else:
            rospy.loginfo("The robot is not in a start zone")

    

       

if __name__ == '__main__': 
    publisher_instance = PathFollow() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass
