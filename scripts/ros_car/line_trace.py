#!/usr/bin/env python

import rospy

from drive_controller import RobotDriveController
from right_canny import Rightcanny
from left_canny import Leftcanny
from detect_stopline import StopLine

class Trace():
    def __init__(self):
        self.right = Rightcanny()
        self.left = Leftcanny()
        self.stop = StopLine()
        self.drive_controller = RobotDriveController()
        self.rate = rospy.Rate(20)

    def go_line(self):
        if self.stop.detect: # detect stopline
            self.drive_controller.set_velocity(0)
            rospy.loginfo('waiting for 3 second.....')
            rospy.sleep(3)
            self.drive_controller.set_velocity(1)
        else: # no detect
            self.drive_controller.set_velocity(1)
            
            if self.right.lines is None and self.left.lines is None:
                self.drive_controller.set_velocity(0.5)
                self.drive_controller.set_angular(0)
            elif self.right.lines is None:
                self.drive_controller.set_angular(-0.8)
                self.drive_controller.set_velocity(0.4)
            elif self.left.lines is None:
                self.drive_controller.set_angular(0.8)
                self.drive_controller.set_velocity(0.4)
            else:
                self.drive_controller.set_velocity(0.8)
                self.drive_controller.set_angular(0)
        self.drive_controller.drive()


if __name__ == '__main__':
    rospy.init_node('Tracer')
    line = Trace()
    while not rospy.is_shutdown():
        line.go_line()
        line.rate.sleep()
        rospy.spin()
