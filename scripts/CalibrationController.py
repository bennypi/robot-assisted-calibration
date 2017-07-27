#!/usr/bin/env python

import rospy
import math
import tf
import tf_conversions
import geometry_msgs.msg
import thread
import MovementController
import actionlib

from caltab_detector.msg import CalibrateAction, CalibrateGoal


class CalibrationController(object):
    def __init__(self, sensor_height, sensor_width, focal_length, caltab_height, caltab_width, camera_x, camera_y,
                 camera_z):
        rospy.init_node('calibration_controller')
        rospy.loginfo('Starting CalibrationController')

        self.close_distance_factor = rospy.get_param('/calibration_controller/close_distance_factor', 0.8)
        self.medium_distance_factor = rospy.get_param('/calibration_controller/medium_distance_factor', 0.5)
        self.far_distance_factor = rospy.get_param('/calibration_controller/far_distance_factor', 0.3)
        rospy.loginfo(
            'Seting distance factors to {}, {} and {}'.format(self.close_distance_factor, self.medium_distance_factor,
                                                              self.far_distance_factor))

        self.sensor_height = sensor_height
        self.sensor_width = sensor_width
        self.focal_length = focal_length
        self.caltab_height = caltab_height
        self.caltab_width = caltab_width
        self.close_distance = 0
        self.medium_distance = 0
        self.far_distance = 0
        self.close_positions_camera = []
        self.medium_positions_camera = []
        self.far_positions_camera = []
        self.close_positions_world = []
        self.medium_positions_world = []
        self.far_positions_world = []
        self.camera_x = camera_x
        self.camera_y = camera_y
        self.camera_z = camera_z
        self.robot_x = 0
        self.robot_y = 0
        self.robot_z = 0.45
        self.quaternion = geometry_msgs.msg.Quaternion()

        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.calibrate_client = actionlib.SimpleActionClient('Calibrate', CalibrateAction)
        self.calibrate_client.wait_for_server()

        rospy.loginfo('Finished initialization of CalibrationController')

    def calculate_distance(self, decrease_factor):
        return self.caltab_height * self.focal_length / (self.sensor_height * decrease_factor)

    def fov_height_for_distance(self, distance):
        return self.sensor_height * distance / self.focal_length

    def fov_width_for_distance(self, distance):
        return self.sensor_width * distance / self.focal_length

    def calculate_all_distances(self):
        self.close_distance = self.calculate_distance(self.close_distance_factor)
        self.medium_distance = self.calculate_distance(self.medium_distance_factor)
        self.far_distance = self.calculate_distance(self.far_distance_factor)

    def calculate_postions_in_camera_frame(self):
        self.calculate_all_distances()

        self.close_positions_camera = [[self.close_distance, 0, 0]]

        fov_height = self.fov_height_for_distance(self.medium_distance)
        fov_width = self.fov_width_for_distance(self.medium_distance)

        # first row
        self.medium_positions_camera.append(
            [self.medium_distance, -(fov_width / 2 - self.caltab_width / 2), fov_height / 2 - self.caltab_height / 2])
        self.medium_positions_camera.append(
            [self.medium_distance, fov_width / 2 - self.caltab_width / 2, fov_height / 2 - self.caltab_height / 2])

        # second row
        self.medium_positions_camera.append(
            [self.medium_distance, -(fov_width / 2 - self.caltab_width / 2),
             -(fov_height / 2 - self.caltab_height / 2)])
        self.medium_positions_camera.append(
            [self.medium_distance, fov_width / 2 - self.caltab_width / 2, -(fov_height / 2 - self.caltab_height / 2)])

        fov_height = self.fov_height_for_distance(self.far_distance)
        fov_width = self.fov_width_for_distance(self.far_distance)

        # first row
        self.far_positions_camera.append(
            [self.far_distance, -(fov_width / 2 - self.caltab_width / 2), fov_height / 2 - self.caltab_height / 2])
        self.far_positions_camera.append([self.far_distance, 0, fov_height / 2 - self.caltab_height / 2])
        self.far_positions_camera.append(
            [self.far_distance, fov_width / 2 - self.caltab_width / 2, fov_height / 2 - self.caltab_height / 2])

        # second row
        self.far_positions_camera.append(
            [self.far_distance, -(fov_width / 2 - self.caltab_width / 2), 0])
        self.far_positions_camera.append([self.far_distance, 0, 0])
        self.far_positions_camera.append(
            [self.far_distance, fov_width / 2 - self.caltab_width / 2, 0])

        # third row
        self.far_positions_camera.append(
            [self.far_distance, -(fov_width / 2 - self.caltab_width / 2), -(fov_height / 2 - self.caltab_height / 2)])
        self.far_positions_camera.append(
            [self.far_distance, 0, -(fov_height / 2 - self.caltab_height / 2)])
        self.far_positions_camera.append(
            [self.far_distance, fov_width / 2 - self.caltab_width / 2, -(fov_height / 2 - self.caltab_height / 2)])

    def transform_camera_pose_to_world_pose(self):
        for pose in self.close_positions_camera:
            self.close_positions_world.append(self.get_world_pose_for_camera_pose(pose))

        for pose in self.medium_positions_camera:
            self.medium_positions_world.append(self.get_world_pose_for_camera_pose(pose))

        for pose in self.far_positions_camera:
            self.far_positions_world.append(self.get_world_pose_for_camera_pose(pose))

    def get_camera_orientation(self):
        vector_x = self.robot_x - self.camera_x
        vector_y = self.robot_y - self.camera_y
        vector_z = self.robot_z - self.camera_z

        # Calculate radians without offset
        yaw = math.atan2(vector_y, vector_x)
        pitch = -math.asin(vector_z)

        self.quaternion = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0, pitch, yaw))

    def thread_publisher(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform((self.camera_x, self.camera_y, self.camera_z),
                                           (self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w),
                                           rospy.Time.now(),
                                           'camera', 'world')
            rate.sleep()

    def publish_camera_frame(self):
        controller.get_camera_orientation()
        thread.start_new_thread(self.thread_publisher, ())
        # wait for transformation to be published
        rospy.sleep(1)

    def get_world_pose_for_camera_pose(self, pose):
        camera_point = geometry_msgs.msg.PointStamped()
        camera_point.header.stamp = rospy.Time.now()
        camera_point.header.frame_id = 'camera'
        camera_point.point.x = pose[0]
        camera_point.point.y = pose[1]
        camera_point.point.z = pose[2]

        rospy.Rate(10).sleep()
        world_point = self.listener.transformPoint('world', camera_point)

        return [world_point.point.x, world_point.point.y, world_point.point.z]


if __name__ == '__main__':
    controller = CalibrationController(0.00327, 0.00585, 0.0037, 0.210, 0.297, -0.7, 0.8, 0.4)
    movement_controller = MovementController.MovementController("/MoveArm", "/FindCaltab")
    controller.calculate_postions_in_camera_frame()
    controller.publish_camera_frame()

    controller.transform_camera_pose_to_world_pose()

    for position in controller.close_positions_world:
        movement_controller.execute_different_orientations(position)
    for position in controller.medium_positions_world:
        movement_controller.execute_different_orientations(position)

    for position in controller.far_positions_world:
        movement_controller.execute_different_orientations(position)

        # goal = CalibrateGoal()
        # goal.goal = 0
        # controller.calibrate_client.send_goal(goal)
        # print 'goal sent'
        # controller.calibrate_client.wait_for_result()
        # print controller.calibrate_client.get_result()
