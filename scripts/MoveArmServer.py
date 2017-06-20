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

    def __init__(self, name, caltab_x, caltab_y, caltab_z, eef_roll, eef_pitch, eef_yaw):
        self.action_name = name
        self.caltab_position_x = caltab_x
        self.caltab_position_y = caltab_y
        self.caltab_position_z = caltab_z
        self.eef_roll = eef_roll
        self.eef_pitch = eef_pitch
        self.eef_yaw = eef_yaw

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
        self.group.set_goal_orientation_tolerance(0.01)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_planning_time(2)

    def execute_cb(self, goal):
        pose_as_string = ', '.join(str(v) for v in goal.pose)
        rospy.loginfo('Received MoveArmAction with posegoal: ' + pose_as_string)

        pose = geometry_msgs.msg.Pose()
        pose.position.x = goal.pose[0]
        pose.position.y = goal.pose[1]
        pose.position.z = goal.pose[2]

        success = self.plan_and_execute(pose, goal.additional_yaw, goal.additional_pitch)
        
        self.result.motion_successful = success
        self.action_server.set_succeeded(self.result)

    def find_quaternion_for_position(self, endeffector_position, additional_yaw, additional_pitch):
        vector = geometry_msgs.msg.Point()
        vector.x = self.caltab_position_x - endeffector_position.x
        vector.y = self.caltab_position_y - endeffector_position.y
        vector.z = self.caltab_position_z - endeffector_position.z

        # Calculate radians without offset
        yaw = math.atan2(vector.y, vector.x)
        yaw += math.radians(additional_yaw)

        # Add offset if eef should not point directly to target
        pitch = -math.asin(vector.z)
        pitch += math.radians(additional_pitch)

        # Add offset if tool is not parallel to eef
        roll = math.radians(self.eef_roll)
        pitch += math.radians(self.eef_pitch)
        yaw += math.radians(self.eef_yaw)

        quaternion = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw))
        return quaternion

    def plan_and_execute(self, pose, additional_yaw, additional_pitch):
        q = self.find_quaternion_for_position(pose.position, additional_yaw, additional_pitch)
        i = geometry_msgs.msg.Quaternion()

        i.x = 0
        i.y = 0
        i.z = 0
        i.w = 1

        pose.orientation = q

        self.group.set_pose_target(pose)
        trajectory_list = []
        for i in range(0, 5):
            plan = self.group.plan()
            trajectory_list = trajectory_list + [plan]
            rospy.loginfo("Points: " + str(len(plan.joint_trajectory.points)))
            # raw_input("Press Enter to continue...")

        print str(len(trajectory_list))
        shortest_trajectory = -1
        index = -1
        valid_trajectory_found = False
        for idx, p in enumerate(trajectory_list):
            if len(p.joint_trajectory.points) > 1 and not valid_trajectory_found:
                shortest_trajectory = len(p.joint_trajectory.points)
                index = idx
                valid_trajectory_found = True
            elif len(p.joint_trajectory.points) > 0 and valid_trajectory_found:
                if len(p.joint_trajectory.points) < shortest_trajectory:
                    shortest_trajectory = len(p.joint_trajectory.points)
                    index = idx

        if not valid_trajectory_found:
            print 'No plan found after five tries, aborting.'
            return False

        rospy.loginfo('Executing plan')
        self.group.execute(trajectory_list[index])
        rospy.loginfo('Execution finished')
        return True

if __name__ == '__main__':
    rospy.init_node('MoveArm')
    server = MoveArmServer(rospy.get_name(), 1, 0, 0.5, 0, 0, 90)
    rospy.spin()
