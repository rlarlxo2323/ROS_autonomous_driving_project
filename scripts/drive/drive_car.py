#!/usr/bin/env python

import cv2
import numpy
import cv_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class go_stop(): # driving
    driving_foward = True
    driving_find = True
    driving_stopline = True
    driving_detect_obs = True

    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()

    def drive(self, msg):
        if self.driving_foward:
            self.twist.linear.x = 1.0
            self.cmd_vel_pub.publish(self.twist)
        else:
            self.twist.linear.x = 0.0
            self.cmd_vel_pub.publish(self.twist)

    # def image_callback(self, msg):
    #     self.twist.linear.x = 0.0
    #     self.cmd_vel_pub.publish(self.twist)
    #
    # def go(self, msg):
    #     self.twist.linear.x = 1.0
    #     self.cmd_vel_pub.publish(self.twist)

class Find_Bar(go_stop): # detect blocking bar
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.bar_pub = rospy.Publisher('camera/rgb/image_raw/p2_bar', Image, queue_size=1)
        self.test = rospy.Subscriber('camera/rgb/image_raw', Image, self.test)
        self.twist = Twist()

    def test(self, msg):
        barimage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(barimage, cv2.COLOR_BGR2HSV)
        lower_red = numpy.array([0, 30, 30])
        upper_red = numpy.array([10, 255, 130])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        h, w, d = barimage.shape
        search_top = 1
        search_bot = 3 * h / 4
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[0:h, 0:250] = 0

        M = cv2.moments(mask)

        rospy.loginfo(self.driving_foward)
        go_stop.drive(self, msg)

        if go_stop.driving_find:
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(barimage, (cx, cy), 10, (255, 0, 0), -1)
                go_stop.driving_foward = False
                go_stop.driving_stopline = False
                go_stop.driving_find = True
                go_stop.driving_detect_obs = False
            else:
                rospy.loginfo(self.driving_foward)
                go_stop.driving_foward = True
                go_stop.driving_stopline = True
                go_stop.driving_find = False
                go_stop.driving_detect_obs = True


class Find_obstacle(go_stop): # detect obstacle
    def __init__(self):
        self.range_ahead = 0
        self.range_right = 0
        self.detect_obstacle_pub = rospy.Publisher('obstacle_pub', Bool, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()

    def scan_callback(self, msg):
        rospy.loginfo(msg.ranges[180])
        #go_stop.drive(self, msg)
        if go_stop.driving_detect_obs:
            if 0 < msg.ranges[180] < 1.5:
                rospy.loginfo("what")
                go_stop.driving_find = False
                go_stop.driving_foward = False
                go_stop.driving_stopline = False
                go_stop.driving_detect_obs = True
            else:
                go_stop.driving_foward = True
                go_stop.driving_find = True
                go_stop.driving_stopline = True
                go_stop.driving_detect_obs = False

class StopLine(go_stop): # detect stop line
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        # detect stop line
        self.stopline_image_pub = rospy.Publisher('camera/rgb/image_raw/stopline', Image, queue_size=1)
        self.twist = Twist()
        self.stopable = 0 # driving state
        self.linecount = 0 # stop line count

    def image_callback(self, msg):
        stoplineimage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(stoplineimage, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 225])
        upper_white = numpy.array([10, 10, 235])
        # detect only white color stop line
        stoplinemask = cv2.inRange(hsv, lower_white, upper_white)
        h, w, d = stoplineimage.shape
        search_top = 5 * h / 6 + 10
        search_bot = 5 * h / 6 + 30
        stoplinemask[0:search_top, 0:w] = 0
        stoplinemask[search_bot:h, 0:w] = 0
        stoplinemask[0:h, 0:w / 2] = 0
        stoplinemask[0:h, (w / 2 + 20):w] = 0
        M2 = cv2.moments(stoplinemask)  # stop line mask

        if go_stop.driving_stopline:
            if M2['m00'] > 0:  # detect stop line
                if self.stopable == 0: # car is driving
                    if self.linecount == 0: # detect first stop line
                        cx = int(M2['m10'] / M2['m00'])
                        cy = int(M2['m01'] / M2['m00'])
                        cv2.circle(stoplineimage, (cx, cy), 10, (255, 0, 0), -1)
                        self.stopable = 1
                        self.linecount += 1
                        go_stop.driving_foward = True
                    else: # detect stop line
                        cx = int(M2['m10'] / M2['m00'])
                        cy = int(M2['m01'] / M2['m00'])
                        cv2.circle(stoplineimage, (cx, cy), 10, (255, 0, 0), -1)
                        go_stop.driving_foward = False
                        rospy.loginfo('stop for 3 second')
                        self.stopable = 1
                        self.linecount += 1
                        rospy.loginfo('linecount = %d', self.linecount)
                        rospy.sleep(3.0)
                else:
                    cx = int(M2['m10'] / M2['m00'])
                    cy = int(M2['m01'] / M2['m00'])
                    cv2.circle(stoplineimage, (cx, cy), 10, (255, 0, 0), -1)
                    go_stop.driving_foward = True
            else: # no detect stop line
                self.stopable = 0
                go_stop.driving_foward = True

            stoplineimage_msg = self.bridge.cv2_to_imgmsg(stoplineimage, 'bgr8')
            self.stopline_image_pub.publish(stoplineimage_msg)

def main():
    rospy.init_node('Drive_bot')
    driving_bot = Find_obstacle()
    driving_bot = Find_Bar()
    driving_bot = StopLine()
    rospy.spin()

if __name__ == "__main__":
    main()