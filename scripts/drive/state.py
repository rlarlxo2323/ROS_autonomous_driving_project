#!/usr/bin/env python

import rospy
from smach import State
from drive_controller import RobotDriveController
from detect_blockbar import Find_Bar


class ReadyToStart(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.drive_controller = RobotDriveController()

    def execute(self, ud):
        rospy.loginfo("loading")
        rospy.sleep(rospy.Duration(1))
        return 'success'


class DetectedBlockingBar(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.detect_blocking_bar = Find_Bar()

    def execute(self, ud):
        rospy.loginfo("blocking bar is detect")
        while True:
            if self.detect_blocking_bar.detect:
                rospy.loginfo("blocking bar is open")
                self.detect_blocking_bar.drive()
                return 'success'

