#!/usr/bin/env python2

""" Exploring basics of subscribing to messages inside of a ROS node """

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy

def process_point(msg):
  print(msg.point)

rospy.init_node('my_subscriber')
sub = rospy.Subscriber('/cool_point', PointStamped, process_point)

r = rospy.Rate(2) # hertz

while not rospy.is_shutdown():
  
  r.sleep()


print('Node is finished')