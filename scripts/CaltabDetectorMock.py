#!/usr/bin/env python

import rospy
import actionlib
from caltab_detector.msg import FindCaltabResult, FindCaltabAction, CalibrateAction, CalibrateActionResult


class CaltabDetectorMock(object):
    """
    This class acts as a mock for the find_caltab action. It will always return true.
    """
    def __init__(self, action_name):
        self.action_name = action_name

        # Initialize the actionserver
        self.result = FindCaltabResult()
        self.action_server = actionlib.SimpleActionServer(self.action_name,
                                                          FindCaltabAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()
        rospy.loginfo('CaltabDetectorMock finished initialization')

    def execute_cb(self, goal):
        rospy.loginfo('CaltabDetectorMock received FindCaltabGoal')
        rospy.sleep(1)
        self.result.result = True
        self.action_server.set_succeeded(self.result)

class CalibrateMock(object):
    """
    This class acts as a mock for the calibrate action. It will always return an empty result.
    """
    def __init__(self, action_name):
        self.action_name = action_name

        # Initialize the actionserver
        self.result = CalibrateActionResult()
        self.action_server = actionlib.SimpleActionServer(self.action_name,
                                                          CalibrateAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()
        rospy.loginfo('CalibrateMock finished initialization')

    def execute_cb(self, goal):
        rospy.loginfo('CaltabDetectorMock received FindCaltabGoal')
        rospy.sleep(1)
        self.action_server.set_succeeded(self.result)


if __name__ == '__main__':
    # Create the two mock objects and then start spinning
    rospy.init_node('CaltabDetectorMock')
    mock = CaltabDetectorMock('/FindCaltab')
    mock = CalibrateMock('/Calibrate')
    rospy.spin()
