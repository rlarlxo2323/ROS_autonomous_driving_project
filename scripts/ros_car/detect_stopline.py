#!/usr/bin/env python

import cv2
import numpy
import numpy as np
import cv_bridge
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from drive_controller import RobotDriveController
from geometry_msgs.msg import Twist

class StopLine:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.rate = rospy.Rate(20)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.stopline_image_pub = rospy.Publisher('camera/rgb/image_raw/stopline', Image, queue_size=1)#detect stopline
        self.twist = Twist()
        self.drive_controller = RobotDriveController()
        self.detect = True # detect stopline
        self.stopable = 0 # driving
        self.linecount = 0       #stopline count

    def get_linecount(self):
        return self.linecount

    def image_callback(self, msg):
        stoplineimage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(stoplineimage, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 225])
        upper_white = numpy.array([10, 10, 235])
        stoplinemask = cv2.inRange(hsv, lower_white, upper_white)  # color_into_white_color
        h, w, d = stoplineimage.shape
        search_top = 5 * h / 6 + 10
        search_bot = 5 * h / 6 + 30
        stoplinemask[0:search_top, 0:w] = 0
        stoplinemask[search_bot:h, 0:w] = 0
        stoplinemask[0:h, 0:w/2] = 0
        stoplinemask[0:h, (w/2+20):w] = 0
        M2 = cv2.moments(stoplinemask) # stopline mask

        if M2['m00'] > 0:   #fisrt detect
            if self.stopable == 0:   # driving                 # 0: True, 1:False
                #if self.linecount == 0 or self.linecount == 2 or self.linecount == 3 or self.linecount == 5 or self.linecount == 9:
                if self.linecount == 0:
                    cx = int(M2['m10'] / M2['m00'])
                    cy = int(M2['m01'] / M2['m00'])
                    cv2.circle(stoplineimage, (cx, cy), 10, (255, 0, 0), -1)
                    self.stopable = 1 # stop
                    self.linecount += 1
                    #self.detect = False
                else: #1,4......
                    cx = int(M2['m10'] / M2['m00'])
                    cy = int(M2['m01'] / M2['m00'])
                    cv2.circle(stoplineimage, (cx, cy), 10, (255, 0, 0), -1)
                    #self.twist.linear.x = 0.0
                    self.detect = True
                    #rospy.sleep(3.0)
                    #rospy.loginfo('wating...............');
                    #self.detect = False
                    # END CONTROL
                    self.stopable = 1 # stop
                    self.linecount += 1
            else: #next detect stopable=1
                cx = int(M2['m10'] / M2['m00'])
                cy = int(M2['m01'] / M2['m00'])
                cv2.circle(stoplineimage, (cx, cy), 10, (255, 0, 0), -1)
                #self.twist.linear.x = 0.8
                self.detect = False
        else: #No detect
            self.stopable = 0 # driving
            #self.twist.linear.x = 0.8
            self.detect = False

        self.cmd_vel_pub.publish(self.twist)
        stoplineimage_msg = self.bridge.cv2_to_imgmsg(stoplineimage, 'bgr8')
        self.stopline_image_pub.publish(stoplineimage_msg) #publish


        rospy.loginfo('stopable = %d', self.stopable)
        rospy.loginfo('linecount = %d', self.linecount)


    def drive(self):
        if self.detect:
            self.drive_controller.set_velocity(0) # stop
        else:
            self.drive_controller.set_velocity(1) # drive
        self.drive_controller.drive()

if __name__ == '__main__':
    rospy.init_node('Tracer')
    detect = StopLine()
    rospy.spin()
    #while not rospy.is_shutdown():
    #    detect.drive()
        #detect.rate.sleep()