#!/usr/bin/env python

import sys
import math
import rospy
import tf_conversions
import moveit_commander
import geometry_msgs.msg
import actionlib
import robot_assisted_calibration.msg


class MovementHandler(object):
    """
    This class acts as a additional abstraction layer to move the arm. 
    
    An action server is provided to move the arm to a given position. Additional values for the yaw and pitch for every 
    position can also be defined. This class will calculate the orientation for the endeffector and will add the yaw
    and pitch values. The roll value is also calculated so that the caltab will point from the robot away.
    
    This class only determines the position and orientation for the endeffector. The path planning and movement is
    done by MoveIt!.
    
    NOTE: It is not recommended to specify the orientation of the calibration object in relation to the tool link 
    with the eef_roll, eef_pitch and eef_yaw parameters. Instead define the orientation in the URDF.
    """

    def __init__(self, name, eef_roll, eef_pitch, eef_yaw):
        self.action_name = name
        self.camera_position_x = rospy.get_param('/robot_assisted_calibration/camera_position/x')
        self.camera_position_y = rospy.get_param('/robot_assisted_calibration/camera_position/y')
        self.camera_position_z = rospy.get_param('/robot_assisted_calibration/camera_position/z')
        self.eef_roll = eef_roll
        self.eef_pitch = eef_pitch
        self.eef_yaw = eef_yaw

        # Initialize the actionserver
        self.result = robot_assisted_calibration.msg.MoveArmResult()
        self.action_server = actionlib.SimpleActionServer(self.action_name,
                                                          robot_assisted_calibration.msg.MoveArmAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()

        # Initialize the moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_goal_orientation_tolerance(0.01)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_planning_time(2)
        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)
        self.group.set_planner_id("RRTConnectkConfigDefault")

        rospy.loginfo('Client Side Path Planning Node started')

    def execute_cb(self, goal):
        """
        This function will be called when a new action goal is received.
        
        :param goal: The position goal with additional orientation values
        :type goal: robot_assisted_calibration.msg.MoveArmGoal
        """

        pose_as_string = ', '.join(str(v) for v in goal.pose)
        rospy.loginfo('Received MoveArmAction with posegoal: %s, yaw: %d, pitch: %d, roll: %d', pose_as_string,
                      goal.additional_yaw, goal.additional_pitch, goal.additional_roll)

        # Create a Pose object
        pose = geometry_msgs.msg.Pose()
        pose.position.x = goal.pose[0]
        pose.position.y = goal.pose[1]
        pose.position.z = goal.pose[2]

        # Calculate the orientation, do the planning and try to move the arm. The outcome will be returned
        success = self.plan_and_execute(pose, goal.additional_yaw, goal.additional_pitch)

        self.result.motion_successful = success
        rospy.loginfo('move_arm action finished with status %s', success)
        self.action_server.set_succeeded(self.result)

    def find_quaternion_for_position(self, endeffector_position, additional_yaw, additional_pitch):
        """
        This function will find the orientation for a given position.
        
        First the orientation is calculated so that the calibration object is orthogonal to the camera. Then the 
        additional yaw and pitch is added.
        
        :param endeffector_position: The position for the endeffector
        :type endeffector_position: geometry_msgs.msg.Pose.Position
        :param additional_yaw: The additional yaw value
        :type additional_yaw: int
        :param additional_pitch: The additional pitch value
        :type additional_pitch: int
        :return: The quaternion for the given position
        :rtype: geometry_msgs.msg.Quaternion
        """

        # Create a vector from the endeffector position to the camera position
        vector = geometry_msgs.msg.Point()
        vector.x = self.camera_position_x - endeffector_position.x
        vector.y = self.camera_position_y - endeffector_position.y
        vector.z = self.camera_position_z - endeffector_position.z

        # Calculate the yaw in radian
        yaw = math.atan2(vector.y, vector.x)
        yaw += math.radians(additional_yaw)

        # Calculate the pitch in radian
        pitch = -math.asin(vector.z)
        pitch += math.radians(additional_pitch)

        # Calculate the roll value for the caltab
        # todo: This does not work for every setup
        val = (float(self.camera_position_y) / float(self.camera_position_x)) * float(endeffector_position.x)
        if val >= endeffector_position.y:
            roll = math.radians(0)
        else:
            roll = math.radians(180)

        # Also add additional roll if specified by client
        # NOTE: You should define the caltab orientation in the URDF. Using these values is not recommended.
        roll += math.radians(self.eef_roll)
        pitch += math.radians(self.eef_pitch)
        yaw += math.radians(self.eef_yaw)

        # Create a Quaternion from the euler angles
        quaternion = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw))
        return quaternion

    def plan_and_execute(self, pose, additional_yaw, additional_pitch):
        """
        This method calculates the orientation, tries to find a valid plan and if at least one is found, the shortest
        plan is executed.
        
        :param pose: The position for the endeffector
        :type pose: geometry_msgs.msg.Pose
        :param additional_yaw: The additional yaw value
        :type additional_yaw: int
        :param additional_pitch: The additional pitch value
        :type additional_pitch: int
        :return: true if the motion was successful, false if no plan was found or the motion was aborted. 
        :rtype: bool
        """

        # First calculate the orientation
        q = self.find_quaternion_for_position(pose.position, additional_yaw, additional_pitch)

        # Add this calculation to the pose
        pose.orientation = q
        self.group.set_pose_target(pose)

        # Try to find a plan five times
        trajectory_list = []
        for i in range(0, 5):
            plan = self.group.plan()
            trajectory_list = trajectory_list + [plan]
            rospy.loginfo("Number of points: " + str(len(plan.joint_trajectory.points)))

        # Check if a valid trajectory was found by MoveIt!. If multiple plans are available, find the shortest.
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
            rospy.logerr('No plan found after five tries, aborting.')
            return False

        rospy.logdebug('Executing plan')
        if self.group.execute(trajectory_list[index]):
            rospy.logdebug('Execution finished')
            return True
        else:
            rospy.logdebug('Execution failed')
            return False


if __name__ == '__main__':
    rospy.init_node('move_arm')
    server = MovementHandler('move_arm', 0, 0, 0)
    rospy.spin()
