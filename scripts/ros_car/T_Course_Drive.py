#!/usr/bin/env python

import cv2
import numpy

import cv_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class tcoursedrive:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('my_right_camera/rgb/image_raw', Image, self.t_drive)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()

    def t_drive(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 220])
        upper_white = numpy.array([0, 0, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = image.shape
        search_top = 3 * h/4
        search_bot = 3 * h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

            err = cx - w / 2
            self.twist.linear.x = 1.0
            self.twist.angular.z = -float(err) / 300
            self.cmd_vel_pub.publish(self.twist)

if __name__ == "__main__":
    rospy.init_node('tcourse')
    tdrive = tcoursedrive()
    rospy.spin()

