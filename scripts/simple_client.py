#!/usr/bin/env python

import roslib

roslib.load_manifest('caltab_detector')
import rospy
import actionlib

from caltab_detector.msg import FindCaltabAction, FindCaltabGoal, CalibrateAction, CalibrateGoal
from robot_assisted_calibration.msg import MoveArmAction, MoveArmGoal


def caltab_detector_client():
    # find_caltab_client = actionlib.SimpleActionClient('/FindCaltab', FindCaltabAction)
    # find_caltab_client.wait_for_server()
    # print 'Waiting for Server finished'
    # for i in range(0, 5):
    #     goal = FindCaltabGoal()
    #     goal.retries = 10
    #     find_caltab_client.send_goal(goal)
    #     print 'SendGoal'
    #     find_caltab_client.wait_for_result()
    #     print find_caltab_client.get_result()
    #     rospy.sleep(5)
    # print 'Finished finding caltabs'
    calibrate_client = actionlib.SimpleActionClient('Calibrate', CalibrateAction)
    calibrate_client.wait_for_server()
    print 'server available'
    goal = CalibrateGoal()
    goal.goal = 0
    calibrate_client.send_goal(goal)
    print 'goal sent'
    calibrate_client.wait_for_result()
    print calibrate_client.get_result()

def move_arm_client(poses, additional_yaw, additional_pitch, additional_roll):
    move_arm_client = actionlib.SimpleActionClient('MoveArm', MoveArmAction)
    move_arm_client.wait_for_server()
    print 'Waiting for Server finished'
    goal = MoveArmGoal()
    goal.pose = poses
    goal.additional_yaw = additional_yaw
    goal.additional_pitch = additional_pitch
    goal.additional_roll = additional_roll
    move_arm_client.send_goal(goal)
    print 'SendGoal'
    move_arm_client.wait_for_result()
    print move_arm_client.get_result()



if __name__ == '__main__':
    rospy.init_node('ActionClient')
    print 'Node started'

    caltab_detector_client()
    # move_arm_client([-0.55, 0.4, 0.45], 0, 0, 0)
    # print 'First pose'
    # raw_input('asdfasfa')
    # move_arm_client([-0.35, 0, 0.5], 10, 10)
    # print 'Second pose'
    # raw_input('asdfasfa')
    # move_arm_client([-0.35, 0, 0.5], -10, -10)
    # print 'Third pose'
