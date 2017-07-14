#!/usr/bin/env python

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

    def take_picture_with_orientation(self, pose, additional_yaw, additional_pitch, additional_roll):
        # raw_input('Waiting for user input to move to next pose')
        movement_goal = MoveArmGoal()
        movement_goal.pose = pose
        movement_goal.additional_yaw = additional_yaw
        movement_goal.additional_pitch = additional_pitch
        movement_goal.additional_roll = additional_roll
        self.move_arm_client.send_goal(movement_goal)
        rospy.loginfo('Sent MoveArmGoal with yaw: %d, pitch: %d and roll: %d', additional_yaw, additional_pitch,
                      additional_roll)
        self.move_arm_client.wait_for_result()
        if self.move_arm_client.get_result().motion_successful is False:
            rospy.logerr('Cannot move to specified orientation')
            return 0

        find_caltab_goal = FindCaltabGoal()
        find_caltab_goal.retries = 3
        self.find_caltab_client.send_goal(find_caltab_goal)
        self.find_caltab_client.wait_for_result()
        if self.find_caltab_client.get_result().result is False:
            rospy.logerr('No caltab detected')
            return 0
        rospy.loginfo('Caltab detected')
        return 1

    def execute_different_orientations(self, pose, additional_roll):
        if self.take_picture_with_orientation(pose, 0, 0, additional_roll) is 0:
            rospy.logerr(
                'Cannot find the caltab on the first try. Please check distance and orientation of camera and caltab')
            rospy.logerr('x: {}, y: {}, z: {}, roll: {}'.format(pose[0], pose[1], pose[2], additional_roll))
            return 0

        pictures = 1

        # for angle in range(10, 50, 10):
        #     pictures += self.take_picture_with_orientation(pose, angle, 0, additional_roll)
        #
        # for angle in range(-10, -50, -10):
        #     pictures += self.take_picture_with_orientation(pose, angle, 0, additional_roll)
        #
        # for angle in range(10, 50, 10):
        #     pictures += self.take_picture_with_orientation(pose, 0, angle, additional_roll)
        #
        # for angle in range(-10, -50, -10):
        #     pictures += self.take_picture_with_orientation(pose, 0, angle, additional_roll)

        rospy.loginfo('Took %d pictures', pictures)
        return pictures


if __name__ == '__main__':
    rospy.init_node('MovementController')
    controller = MovementController("/MoveArm", "/FindCaltab")

    total_pictures = 0

    total_pictures += controller.execute_different_orientations([-0.50, -0.50, 0.45], 0)  # close distance
    total_pictures += controller.execute_different_orientations([-0.45, -0.15, 0.3], 180)  # left low middle distance
    total_pictures += controller.execute_different_orientations([-0.45, -0.15, 0.45], 180)  # left top middle distance
    total_pictures += controller.execute_different_orientations([-0.35, -0.15, 0.25], 0)  # middle low middle distance
    total_pictures += controller.execute_different_orientations([-0.45, -0.15, 0.45], 0)  # middle top middle distance
    total_pictures += controller.execute_different_orientations([-0.25, -0.45, 0.3], 0)  # right low middle distance
    total_pictures += controller.execute_different_orientations([-0.25, -0.45, 0.45], 0)  # right top middle distance

    rospy.loginfo(total_pictures)
