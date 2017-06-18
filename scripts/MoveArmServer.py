#!/bin/bash

import sys
import math
import rospy
import tf_conversions
import moveit_commander
import geometry_msgs.msg
import actionlib
import robot_assisted_calibration.msg


class MoveArmServer(object):
    # create messages that are used to publish feedback/result
    feedback = robot_assisted_calibration.msg.MoveArmFeedback()
    result = robot_assisted_calibration.msg.MoveArmResult()

    def __init__(self, name):
        self.action_name = name
        self.action_server = actionlib.SimpleActionServer(self.action_name,
                                                          robot_assisted_calibration.msg.MoveArmAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()

    def execute_cb(self, goal):
        rospy.loginfo('Received MoveArmAction with posegoal:' + goal.pose)


if __name__ == '__main__':
    rospy.init_node('MoveArm')
    server = MoveArmServer(rospy.get_name)
    rospy.spin()
