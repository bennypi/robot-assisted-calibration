#! /usr/bin/env python

import roslib

roslib.load_manifest('caltab_detector')
import rospy
import actionlib

from caltab_detector.msg import FindCaltabAction, FindCaltabGoal, CalibrateAction, CalibrateGoal

if __name__ == '__main__':
    print 'Node started'
    rospy.init_node('ActionClient')

    find_caltab_client = actionlib.SimpleActionClient('FindCaltab', FindCaltabAction)
    find_caltab_client.wait_for_server()
    print 'Waiting for Server finished'

    for i in range(0, 10):
        goal = FindCaltabGoal()
        goal.retries = 10
        find_caltab_client.send_goal(goal)
        print 'SendGoal'
        find_caltab_client.wait_for_result()
        print find_caltab_client.get_result()
        rospy.sleep(5)

    print 'Finished finding caltabs'
    calibrate_client = actionlib.SimpleActionClient('Calibrate', CalibrateAction)
    calibrate_client.wait_for_server()
    print 'server available'
    goal = CalibrateGoal()
    goal.goal = 0
    calibrate_client.send_goal(goal)
    print 'goal sent'
    calibrate_client.wait_for_result()
    print calibrate_client.get_result()