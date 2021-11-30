#!/usr/bin/env python

import rospy
import state
from smach import StateMachine


class RobotStateMachine(object):
    def __init__(self):
        self.autonomous_drive = StateMachine(outcomes=['success'])

    def drive_robot(self):
        with self.autonomous_drive:
            StateMachine.add('READY_TO_START', state.ReadyToStart(), transitions={'success': 'DETECT_BLOCKING_BAR'})
            StateMachine.add('DETECT_BLOCKING_BAR', state.DetectedBlockingBar(),
                             transitions={'success': 'LINE_TRACE'})
            StateMachine.add('LINE_TRACE', state.LineTrace(), transitions={'success': 'success'})
            self.autonomous_drive.execute()


if __name__ == "__main__":
    rospy.init_node('autonomous_car_test_map_drive')
    robot_state_machine = RobotStateMachine()
    robot_state_machine.drive_robot()
    while not rospy.is_shutdown():
        rospy.spin()
