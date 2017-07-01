#!/bin/bash

import rospy
import actionlib

from caltab_detector.msg import FindCaltabAction, FindCaltabGoal, FindCaltabResult
from robot_assisted_calibration.msg import MoveArmAction, MoveArmGoal, MoveArmResult


class MovementController(object):
    def __init__(self, move_arm_action_name, find_caltab_action_name):
        self.move_arm_action_name = move_arm_action_name
        self.find_caltab_action_name = find_caltab_action_name

        self.find_caltab_client = actionlib.SimpleActionClient(self.find_caltab_action_name, FindCaltabAction)
        self.find_caltab_client.wait_for_server()

        self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_action_name, MoveArmAction)
        self.move_arm_client.wait_for_server()
        rospy.loginfo('Finished initialization of MovementController')

    def take_picture_with_orientation(self, pose, additional_yaw, additional_pitch):
        movement_goal = MoveArmGoal()
        movement_goal.pose = pose
        movement_goal.additional_yaw = additional_yaw
        movement_goal.additional_pitch = additional_pitch
        self.move_arm_client.send_goal(movement_goal)
        rospy.loginfo('Sent MoveArmGoal with yaw: %d and pitch: %d', additional_yaw, additional_pitch)
        self.move_arm_client.wait_for_result()
        if self.move_arm_client.get_result().motion_successful is False:
            rospy.loginfo('Cannot move to specified orientation')
            return 0

        find_caltab_goal = FindCaltabGoal()
        find_caltab_goal.retries = 3
        self.find_caltab_client.send_goal(find_caltab_goal)
        self.find_caltab_client.wait_for_result()
        if self.find_caltab_client.get_result().result is False:
            rospy.loginfo('No caltab detected')
            return 0
        rospy.loginfo('Caltab detected')
        return 1

    def execute_different_orientations(self, pose):
        if self.take_picture_with_orientation(pose, 0, 0) is 0:
            rospy.logerr(
                'Cannot find the caltab on the first try. Please check distance and orientation of camera and caltab')
            return 0

        pictures = 1

        for angle in range(10, 50, 10):
            result = self.take_picture_with_orientation(pose, angle, 0)
            if result is 0:
                break
            pictures += 1

        for angle in range(-10, -50, -10):
            result = self.take_picture_with_orientation(pose, angle, 0)
            if result is 0:
                break
            pictures += 1

        for angle in range(10, 50, 10):
            result = self.take_picture_with_orientation(pose, 0, angle)
            if result is 0:
                break
            pictures += 1

        for angle in range(-10, -50, -10):
            result = self.take_picture_with_orientation(pose, 0, angle)
            if result is 0:
                break
            pictures += 1

        rospy.loginfo('Took %d pictures', pictures)
        return pictures


if __name__ == '__main__':
    rospy.init_node('MovementController')
    controller = MovementController("/MoveArm", "/FindCaltab")

    controller.execute_different_orientations([0.35, 0, 0.5])
