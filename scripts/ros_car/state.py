#!/usr/bin/env python

import rospy
from smach import State
from drive_controller import RobotDriveController
from detect_blocking_bar import Find_Bar
from line_trace import Trace
from detect_stopline import StopLine


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
        current_time = int(rospy.Time.now().to_sec())
        target_time = current_time + 6
        while True:
            if self.detect_blocking_bar.detect:
                rospy.loginfo("blocking bar is open")
                while target_time > int(rospy.Time.now().to_sec()):
                    self.detect_blocking_bar.drive()
                return 'success'


#class DetectedStopLine(State):
 #   def __init__(self):
   #     State.__init__(self, outcomes=['success'])
    #    self.detect_stopline = StopLine()

   # def execute(self, ud):
    #    rospy.loginfo("stopline is detect")
    #    while True:
     #       if self.detect_stopline.detect:
      #          rospy.loginfo("waiting for 3 second...")
      #          rospy.sleep(3)
  #              self.detect_stopline.drive()
     #   return 'success'


class LineTrace(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.line_trace = Trace()

    def execute(self, ud):
        rospy.loginfo("line trace")
        while True:
            self.line_trace.go_line()
        return 'success'