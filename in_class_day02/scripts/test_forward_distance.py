#!/usr/bin/env python2

""" Print distance to object immediately in front of the robot """

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import rospy

def process_scan(msg):
  print(msg.ranges[0])

rospy.init_node('forward_distance')
sub = rospy.Subscriber('/scan', LaserScan, process_scan)

r = rospy.Rate(2) # hertz

while not rospy.is_shutdown():
  
  r.sleep()


print('Node is finished')
