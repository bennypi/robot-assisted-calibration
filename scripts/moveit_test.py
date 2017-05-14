#!/usr/bin/env python

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg

from std_msgs.msg import String


def createPositions():
    global closePose, mediumPose, farPose
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


def plan_and_execute(pose):
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
    plan_and_execute(closePose)
    plan_and_execute(mediumPose)


if __name__ == '__main__':
    try:
        createPositions()
        move_group_python_interface_tutorial()
        move_postions()
    except rospy.ROSInterruptException:
        pass
