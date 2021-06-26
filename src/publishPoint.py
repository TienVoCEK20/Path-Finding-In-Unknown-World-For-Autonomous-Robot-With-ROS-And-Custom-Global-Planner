#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
import tf
import random
import numpy as np

global x,y,z
x = 0.0
y = 0.0
z = 0.0

def callback(msg): 
    goal = PointStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "/map"
    goal.point.x = msg.point.x 
    goal.point.y = msg.point.y
    goal.point.z = msg.point.z
    rospy.loginfo("coordinates:x=%f y=%f" %(point.point.x,point.point.y))
    return goal.point.x,goal.point.y

def listener():
    rospy.init_node('goal_publisher', anonymous=True)
    rospy.point_pub = rospy.Subscriber('/clicked_point', PointStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
