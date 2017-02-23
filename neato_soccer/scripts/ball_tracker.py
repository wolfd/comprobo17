#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')
        # cv2.setMouseCallback('video_window', self.process_mouse_event)

        self.lower_default = (84, 11, 22)
        self.upper_default = (153, 255, 251)

        rospy.on_shutdown(self.stop)

        self.circles = None

    def stop(self):
        """
        Tells the robot to stop moving.
        """
        self.pub.publish(
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        )

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window,
                    'Color (h=%d,s=%d,v=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

    def filter_image(self):
        self.binary_image = cv2.inRange(
            self.hsv_image,
            self.lower_default,
            self.upper_default
        )

    def find_circles(self):
        """ doesn't really work, but code to find circles in an image """
        self.circles = cv2.HoughCircles(self.hsv_image[:,:,1], cv2.HOUGH_GRADIENT, 1.2, 100)
        if self.circles is not None:
            print self.circles

    def find_center_of_ball(self):
        """ finds center of mass of a binary image """
        moments = cv2.moments(self.binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10'] / moments['m00'], moments['m01'] / moments['m00']

            print self.center_x, self.center_y

    def run(self):
        """ The main run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                print self.cv_image.shape

                self.filter_image()
                self.find_center_of_ball()
                # self.find_circles()

                self.drawn_upon = self.cv_image.copy() # copy image to draw on

                cv2.circle(self.drawn_upon, (int(self.center_x), int(self.center_y)), 40, (255, 0, 0), -1)

                # loop over the (x, y) coordinates and radius of the circles
                if self.circles is not None:
                    converted_circles = np.round(self.circles[0, :]).astype('int')
                    for (x, y, radius) in converted_circles:
                        # draw the circle in the output image, then draw a rectangle
                        # corresponding to the center of the circle
                        cv2.circle(self.drawn_upon, (x, y), radius, (0, 255, 0), 4)
                        cv2.rectangle(self.drawn_upon, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                cv2.imshow('video_window', self.binary_image)
                cv2.imshow('drawn_window', self.drawn_upon)
                cv2.waitKey(5)

                # convert the coords in image space to -.5 to .5 range
                turn_rate = -((self.center_x / float(self.cv_image.shape[1])) - 0.5) * 3.0


                turn_msg = Twist(
                    linear=Vector3(1.0 - abs(turn_rate), 0.0, 0.0),
                    angular=Vector3(0.0, 0.0, turn_rate)
                )
                self.pub.publish(turn_msg)



            # start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()