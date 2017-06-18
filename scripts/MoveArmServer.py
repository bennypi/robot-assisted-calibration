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

    def __init__(self, name, caltab_x, caltab_y, caltab_z):
        self.action_name = name
        self.caltab_position_x = caltab_x
        self.caltab_position_y = caltab_y
        self.caltab_position_z = caltab_z

        # Initialize the actionserver
        self.result = robot_assisted_calibration.msg.MoveArmResult()
        self.action_server = actionlib.SimpleActionServer(self.action_name,
                                                          robot_assisted_calibration.msg.MoveArmAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()
        rospy.loginfo('MoveArm actionserver started')

        # Initialize the moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        # robot = moveit_commander.RobotCommander()
        # scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

    def execute_cb(self, goal):
        pose_as_string = ', '.join(str(v) for v in goal.pose)
        rospy.loginfo('Received MoveArmAction with posegoal: ' + pose_as_string)

        pose = geometry_msgs.msg.Pose()
        pose.position.x = goal.pose[0]
        pose.position.y = goal.pose[1]
        pose.position.z = goal.pose[2]

        self.plan_and_execute(pose)
        
        self.result.motion_successful = True
        self.action_server.set_succeeded(self.result)

    def find_quaternion_for_postion(self, endeffector_position):
        vector = geometry_msgs.msg.Point()
        vector.x = self.caltab_position_x - endeffector_position.x
        vector.y = self.caltab_position_y - endeffector_position.y
        vector.z = self.caltab_position_z - endeffector_position.z

        yaw = math.atan2(vector.y, vector.x)
        pitch = -math.asin(vector.z)

        print yaw, pitch

        quaternion = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pitch, yaw))
        print quaternion
        return quaternion

    def plan_and_execute(self, pose):
        q = self.find_quaternion_for_postion(pose.position)
        i = geometry_msgs.msg.Quaternion()

        i.x = 0
        i.y = 0
        i.z = 0
        i.w = 1

        pose.orientation = q
        self.group.set_pose_target(pose)
        plan = self.group.plan()
        count = 0
        while len(plan.joint_trajectory.points) == 0:
            if count == 4:
                print 'No plan found after five tries, aborting.'
                return
            print 'Could not find plan to goal, replanning...'
            plan = self.group.plan()
            count += 1
        self.group.execute(plan)



if __name__ == '__main__':
    rospy.init_node('MoveArm')
    server = MoveArmServer(rospy.get_name(), 1, 0, 0.5)
    rospy.spin()
