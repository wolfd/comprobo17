#!/usr/bin/env python2

""" Exploring basics of creating messages inside of a ROS node """

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy

rospy.init_node('my_first_node')
pub = rospy.Publisher('/cool_point', PointStamped, queue_size=10)


r = rospy.Rate(2) # hertz

while not rospy.is_shutdown():
  #point_message = Point(1.0, 2.0, 0.0)
  point_message = Point(x=1.0, y=2.0)
  header_message = Header(stamp=rospy.Time.now(), frame_id='odom') # or base_link
  point_stamped_message = PointStamped(
    header=header_message,
    point=point_message
  )

  pub.publish(point_stamped_message)

  r.sleep()


print('Node is finished')