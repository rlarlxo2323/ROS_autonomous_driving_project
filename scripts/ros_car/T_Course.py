#!/usr/bin/env python

import cv2
import numpy

import cv_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from drive_controller import RobotDriveController
# from T_Course_Drive import tcoursedrive


class T_Course:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback1)
        self.image_sub2 = rospy.Subscriber('my_left_camera/rgb/image_raw', Image, self.image_callback2)
        self.image_sub3 = rospy.Subscriber('my_right_camera/rgb/image_raw', Image, self.image_callback3)
        # self.image_sub4 = rospy.Subscriber('my_right_camera/rgb/image_raw', Image, self.t_drive)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.count = 0
        self.drive_controller = RobotDriveController()
        # self.tdrive = tcoursedrive()
        self.straight = None
        self.rightt= None
        self.leftt= None
        self.clear = True


    def image_callback1(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([0, 0, 220])
        upper_yellow = numpy.array([0, 0, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # h, w, d = image.shape
        # mask[0:h * 3 / 5, 0:w] = 0
        # mask[h - (h / 8):h, 0:w] = 0
        # mask[0:h, 0:w / 4] = 0
        # mask[0:h, w - (w / 4):w] = 0

        h, w, d = image.shape
        search_top = 5 * h / 6 + 10
        search_bot = 5 * h / 6 + 30
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[0:h,0:w/2] = 0
        mask[0:h,(w/2+20):w] = 0


        M = cv2.moments(mask)
        if M['m00'] > 0:
            print('straight true')
            self.straight = True
        else:
            print('straight false')
            self.straight = False

    def image_callback2(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # lower_yellow = numpy.array([0, 0, 220])
        # upper_yellow = numpy.array([0, 0, 255])
        lower_yellow = numpy.array([0, 0, 220])
        upper_yellow = numpy.array([0, 0, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # h, w, d = image.shape
        # mask[0:h * 3 / 5, 0:w] = 0
        # mask[h - (h / 8):h, 0:w] = 0
        # mask[0:h, 0:w / 4] = 0
        # mask[0:h, w - (w / 4):w] = 0

        h, w, d = image.shape
        search_top = 3 * h/4
        search_bot = 3 * h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # h, w, d = image.shape
        # search_top = 5 * h / 6 + 10
        # search_bot = 5 * h / 6 + 30
        # mask[0:search_top, 0:w] = 0
        # mask[search_bot:h, 0:w] = 0
        # mask[0:h,0:w/2] = 0
        # mask[0:h,(w/2+20):w] = 0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            print('left true')
            self.leftt = True
        else:
            print('left false')
            self.leftt = False

        rospy.loginfo('-----------------')

    def image_callback3(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([0, 0, 220])
        upper_yellow = numpy.array([0, 0, 255])
        # lower_yellow = numpy.array([10, 100, 100])
        # upper_yellow = numpy.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

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

        M = cv2.moments(mask)
        if M['m00'] > 0:
            print('right true')
            self.rightt = True
        else:
            print('right false')
            self.rightt = False


    def drive(self):

        if not self.leftt:
            self.drive_controller.set_angular(0.2)
            self.drive_controller.set_velocity(0.4)
            self.drive_controller.drive()

        if self.straight:
            while self.straight:
                self.drive_controller.set_angular(-1)
                self.drive_controller.set_velocity(0)
                self.drive_controller.drive()
                if self.leftt:
                    while self.leftt:
                        self.drive_controller.set_angular(-1)
                        self.drive_controller.set_velocity(0)
                        self.drive_controller.drive()
                        if not self.straight or not self.rightt:
                            break

        self.drive_controller.set_velocity(0.4)
        self.drive_controller.drive()

if __name__ == '__main__':
    rospy.init_node('test_node1234')
    tcourse = T_Course()
    while not rospy.is_shutdown():
        tcourse.drive()



