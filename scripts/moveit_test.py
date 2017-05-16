#!/usr/bin/env python

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import math
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg

from std_msgs.msg import String


def create_positions():
    global closePose, mediumPose, highPose, leftPose
    closePose = geometry_msgs.msg.Pose()
    closePose.position.x = 0.6
    closePose.position.y = 0
    closePose.position.z = 0.5
    closePose.orientation.x = -0.706690260596
    closePose.orientation.y = -0.707520537936
    closePose.orientation.z = 0.00134327034298
    closePose.orientation.w = 0.00132649993095

    mediumPose = geometry_msgs.msg.Pose()
    mediumPose.position.x = 0.35
    mediumPose.position.y = 0
    mediumPose.position.z = 0.5
    mediumPose.orientation.x = -0.706690260596
    mediumPose.orientation.y = -0.707520537936
    mediumPose.orientation.z = 0.00134327034298
    mediumPose.orientation.w = 0.00132649993095

    highPose = geometry_msgs.msg.Pose()
    highPose.position.x = 0.35
    highPose.position.y = 0
    highPose.position.z = 0.8
    highPose.orientation.x = -0.706690260596
    highPose.orientation.y = -0.707520537936
    highPose.orientation.z = 0.00134327034298
    highPose.orientation.w = 0.00132649993095

    leftPose = geometry_msgs.msg.Pose()
    leftPose.position.x = 0.35
    leftPose.position.y = 0.5
    leftPose.position.z = 0.5
    leftPose.orientation.x = -0.706690260596
    leftPose.orientation.y = -0.707520537936
    leftPose.orientation.z = 0.00134327034298
    leftPose.orientation.w = 0.00132649993095

    global caltab_position
    caltab_position = geometry_msgs.msg.Point()
    caltab_position.x = 1
    caltab_position.y = 0
    caltab_position.z = 0.5


def find_quaternion_for_postion(endeffector_position):
    vector = geometry_msgs.msg.Point()
    vector.x = caltab_position.x - endeffector_position.x
    vector.y = caltab_position.y - endeffector_position.y
    vector.z = caltab_position.z - endeffector_position.z

    yaw = math.degrees(math.atan2(vector.y, vector.x))
    pitch = math.degrees(math.asin(vector.z))

    print yaw, pitch

    quaternion = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pitch, yaw))
    print quaternion
    return quaternion


def plan_and_execute(pose):
    q = find_quaternion_for_postion(pose.position)
    i = geometry_msgs.msg.Quaternion()

    i.x = 0
    i.y = 0
    i.z = 0
    i.w = 1

    pose.orientation = q
    group.set_pose_target(pose)
    plan = group.plan()
    count = 0
    while len(plan.joint_trajectory.points) == 0:
        if count == 4:
            print 'No plan found after five tries, aborting.'
            return
        print 'Could not find plan to goal, replanning...'
        plan = group.plan()
        count += 1
    group.execute(plan)


def move_group_python_interface_tutorial():
    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    global group
    group = moveit_commander.MoveGroupCommander("manipulator")


def move_postions():
    # plan_and_execute(closePose)
    plan_and_execute(leftPose)
    raw_input('Execution paused, press any key to resume')
    plan_and_execute(highPose)


if __name__ == '__main__':
    try:
        create_positions()
        move_group_python_interface_tutorial()
        move_postions()
    except rospy.ROSInterruptException:
        pass
