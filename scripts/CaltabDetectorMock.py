#!/usr/bin/env python

import rospy
from random import randint
import actionlib
from caltab_detector.msg import FindCaltabResult, FindCaltabAction


class CaltabDetectorMock(object):
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
        rospy.sleep(randint(1, 3))
        self.result.result = True
        self.action_server.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node('CaltabDetectorMock')
    mock = CaltabDetectorMock('/FindCaltab')
    rospy.spin()
