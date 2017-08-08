#!/usr/bin/env python

import rospy
import actionlib

from robot_assisted_calibration.msg import MoveArmAction, MoveArmGoal, FindCalibrationObjectAction, \
    FindCalibrationObjectGoal


class MovementController(object):
    """
    This class controls the movement of the arm and the input for the computer vision node.
    
    The function that should be used by other classes is the execute_different_orientations function. For a given
    position, this function will rotate the endeffector with different yaw and pitch values. For every new orientation
    the find_caltab action is called so that the computer vision node can evaluate the pictures.
    """

    def __init__(self, move_arm_action_name, find_caltab_action_name):
        self.latency = rospy.get_param('/robot_assisted_calibration/latency', 0.5)
        rospy.loginfo('Seting latency to {}'.format(self.latency))

        self.move_arm_action_name = move_arm_action_name
        self.find_caltab_action_name = find_caltab_action_name

        self.find_caltab_client = actionlib.SimpleActionClient(self.find_caltab_action_name,
                                                               FindCalibrationObjectAction)
        self.find_caltab_client.wait_for_server()

        self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_action_name, MoveArmAction)
        self.move_arm_client.wait_for_server()
        rospy.loginfo('Finished initialization of MovementController')

    def take_picture_with_orientation(self, pose, additional_yaw, additional_pitch):
        """
        This method will first send a move_arm action goal with the given position and additional orientation values.
        Then the find_caltab action server is called. 
        
        If both action servers return true, 1 is returned, otherwise 0.
        
        :param pose: The position of the caltab
        :type pose: list of float
        :param additional_yaw: The additional yaw value
        :type additional_yaw: int
        :param additional_pitch: The additional pitch value
        :type additional_pitch: int
        :return: 1 if the caltab was found, 0 otherwise
        :rtype: int
        """

        # Create the move_arm goal message
        movement_goal = MoveArmGoal()
        movement_goal.pose = pose
        movement_goal.additional_yaw = additional_yaw
        movement_goal.additional_pitch = additional_pitch
        self.move_arm_client.send_goal(movement_goal)
        rospy.loginfo('Sent MoveArmGoal with yaw: %d, pitch: %d and roll: %d', additional_yaw, additional_pitch,
                      0)
        self.move_arm_client.wait_for_result()
        if self.move_arm_client.get_result().motion_successful is False:
            rospy.logerr('Cannot move to specified orientation')
            return 0

        # Sleep for a given time if the camera has too much latency
        rospy.sleep(self.latency)

        # Create the find_caltab goal message
        find_caltab_goal = FindCalibrationObjectGoal()
        find_caltab_goal.retries = 3
        self.find_caltab_client.send_goal(find_caltab_goal)
        self.find_caltab_client.wait_for_result()
        if self.find_caltab_client.get_result().result is False:
            rospy.logerr('No caltab detected')
            return 0
        rospy.loginfo('Caltab detected')
        return 1

    def execute_different_orientations(self, pose):
        """
        For a given position, take a picture with and without additional orientation values.
        
        The number pictures in which the caltab was found is returned.
        
        :param pose: The position of the caltab
        :type pose: list of float
        :return: The number of pictures in which the caltab was found
        :rtype: int
        """
        pictures = 0

        pictures += self.take_picture_with_orientation(pose, 0, 0)

        for angle in range(10, 50, 10):
            pictures += self.take_picture_with_orientation(pose, angle, 0)

        for angle in range(-10, -50, -10):
            pictures += self.take_picture_with_orientation(pose, angle, 0)

        for angle in range(10, 50, 10):
            pictures += self.take_picture_with_orientation(pose, 0, angle)

        for angle in range(-10, -50, -10):
            pictures += self.take_picture_with_orientation(pose, 0, angle)

        rospy.loginfo('The caltab was found %d times', pictures)
        return pictures
