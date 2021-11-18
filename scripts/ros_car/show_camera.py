#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("L", 1)
        self.image_sub = rospy.Subscriber('my_left_camera/rgb/image_raw', Image, self.image_callback_1)

        cv2.namedWindow("R", 3)
        self.image_sub = rospy.Subscriber('my_right_camera/rgb/image_raw', Image, self.image_callback_3)

        # cv2.namedWindow("window2", 2)
        # self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_2)

    def image_callback_1(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("L", image)
        cv2.waitKey(3)

    def image_callback_2(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("window2", image)
        cv2.waitKey(3)

    def image_callback_3(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("R", image)
        cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
