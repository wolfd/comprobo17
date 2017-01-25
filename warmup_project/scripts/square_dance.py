#!/usr/bin/env python2

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from neato_node.msg import Bump
import numpy as np
import tf

import rospy

class SquareDance(object):
	def __init__(self):
		super(SquareDance, self).__init__()
		self.left_front_triggered = 0
		self.right_front_triggered = 0

		self.position = None
		self.orientation = None

		self.starting_position = None
		self.starting_orientation = None

		self.running = True

		rospy.init_node('square_dance')

		rospy.Subscriber('/bump', Bump, self.detect_bump)
		rospy.Subscriber('/odom', Odometry, self.update_odometry)
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		rospy.on_shutdown(self.stop)

	def convert_to_euler(self, x, y, z, w):
		quaternion = (x, y, z, w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		return np.array([roll, pitch, yaw])


	def stop(self):
		self.publisher.publish(
			Twist(linear=Vector3(0.0, 0.0, 0.0))
		)

	def detect_bump(self, msg):
		self.left_front_triggered = msg.leftFront
		self.right_front_triggered = msg.rightFront

	def update_odometry(self, msg):
		if self.position is None:
			pos = msg.pose.pose.position
			self.starting_position = np.array([pos.x, pos.y, pos.z])
			quat = msg.pose.pose.orientation
			self.starting_orientation = self.convert_to_euler(quat.x, quat.y, quat.z, quat.w)
		current_pos = msg.pose.pose.position
		self.position = np.array([current_pos.x, current_pos.y, current_pos.z]) - self.starting_position
		current_quat = msg.pose.pose.orientation
		self.orientation = self.convert_to_euler(current_quat.x, current_quat.y, current_quat.z, current_quat.w) - self.starting_orientation
	
	def run(self):
		r = rospy.Rate(50)
		while not rospy.is_shutdown() and self.running:
			#fwd_msg = Twist(linear=Vector3(1.0, 0.0, 0.0))
			#self.publisher.publish(fwd_msg)

			#if self.left_front_triggered == 1 or self.right_front_triggered == 1:
			#	self.running = False

			print(self.position, self.orientation)
			r.sleep()
		
		self.stop()

SquareDance().run()
