#!/usr/bin/env python

import rospy
import math
import tf
import tf_conversions
import geometry_msgs.msg
import thread


class CalibrationController(object):
    def __init__(self, sensor_height, sensor_width, focal_length, caltab_height, caltab_width, camera_x, camera_y,
                 camera_z):
        self.close_distance = 0
        self.medium_distance = 0
        self.far_distance = 0
        self.sensor_height = sensor_height
        self.sensor_width = sensor_width
        self.focal_length = focal_length
        self.caltab_height = caltab_height
        self.caltab_width = caltab_width
        self.close_positions = []
        self.medium_positions = []
        self.far_positions = []
        self.camera_x = camera_x
        self.camera_y = camera_y
        self.camera_z = camera_z
        self.robot_x = 0
        self.robot_y = 0
        self.robot_z = 0.45
        self.quaternion = geometry_msgs.msg.Quaternion()

        rospy.init_node('calibration_controller')
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        rospy.loginfo('Hello world')

    def calculate_distance(self, decrease_factor):
        return self.caltab_height * self.focal_length / (self.sensor_height * decrease_factor)

    def fov_height_for_distance(self, distance):
        return self.sensor_height * distance / self.focal_length

    def fov_width_for_distance(self, distance):
        return self.sensor_width * distance / self.focal_length

    def calculate_all_distances(self):
        self.close_distance = self.calculate_distance(0.8)
        self.medium_distance = self.calculate_distance(0.5)
        self.far_distance = self.calculate_distance(0.3)

    def calculate_postions(self):
        self.calculate_all_distances()

        self.close_positions = [[self.close_distance, 0, 0]]

        fov_height = self.fov_height_for_distance(self.medium_distance)
        fov_width = self.fov_width_for_distance(self.medium_distance)

        # first row
        self.medium_positions.append(
            [self.medium_distance, -(fov_width / 2 - self.caltab_width / 2), fov_height / 2 - self.caltab_height / 2])
        self.medium_positions.append(
            [self.medium_distance, fov_width / 2 - self.caltab_width / 2, fov_height / 2 - self.caltab_height / 2])

        # second row
        self.medium_positions.append(
            [self.medium_distance, -(fov_width / 2 - self.caltab_width / 2),
             -(fov_height / 2 - self.caltab_height / 2)])
        self.medium_positions.append(
            [self.medium_distance, fov_width / 2 - self.caltab_width / 2, -(fov_height / 2 - self.caltab_height / 2)])

        fov_height = self.fov_height_for_distance(self.far_distance)
        fov_width = self.fov_width_for_distance(self.far_distance)

        # first row
        self.far_positions.append(
            [self.far_distance, -(fov_width / 2 - self.caltab_width / 2), fov_height / 2 - self.caltab_height / 2])
        self.far_positions.append([self.far_distance, 0, fov_height / 2 - self.caltab_height / 2])
        self.far_positions.append(
            [self.far_distance, fov_width / 2 - self.caltab_width / 2, fov_height / 2 - self.caltab_height / 2])

        # second row
        self.far_positions.append(
            [self.far_distance, -(fov_width / 2 - self.caltab_width / 2), 0])
        self.far_positions.append([self.far_distance, 0, 0])
        self.far_positions.append(
            [self.far_distance, fov_width / 2 - self.caltab_width / 2, 0])

        # third row
        self.far_positions.append(
            [self.far_distance, -(fov_width / 2 - self.caltab_width / 2), -(fov_height / 2 - self.caltab_height / 2)])
        self.far_positions.append(
            [self.far_distance, 0, -(fov_height / 2 - self.caltab_height / 2)])
        self.far_positions.append(
            [self.far_distance, fov_width / 2 - self.caltab_width / 2, -(fov_height / 2 - self.caltab_height / 2)])

    def get_camera_orientation(self):
        vector_x = self.robot_x - self.camera_x
        vector_y = self.robot_y - self.camera_y
        vector_z = self.robot_z - self.camera_z

        # Calculate radians without offset
        yaw = math.atan2(vector_y, vector_x)
        pitch = -math.asin(vector_z)

        self.quaternion = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0, pitch, yaw))

        rospy.loginfo('transform arguments:')
        rospy.loginfo(self.camera_x)
        rospy.loginfo(self.camera_y)
        rospy.loginfo(self.camera_z)
        rospy.loginfo(self.quaternion)

    def thread_publisher(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform((self.camera_x, self.camera_y, self.camera_z),
                                           (self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w),
                                           rospy.Time.now(),
                                           'camera', 'world')
            rate.sleep()

    def publish_camera_frame(self):
        controller.get_camera_orientation()
        thread.start_new_thread(self.thread_publisher, ())

    def get_world_pose_for_camera_pose(self, x, y, z):
        camera_point = geometry_msgs.msg.PointStamped()
        camera_point.header.stamp = rospy.Time.now()
        camera_point.header.frame_id = 'camera'
        camera_point.point.x = x
        camera_point.point.y = y
        camera_point.point.z = z

        rospy.loginfo(camera_point)
        rospy.Rate(10).sleep()
        world_point = self.listener.transformPoint('world', camera_point)

        rospy.loginfo(world_point)


if __name__ == '__main__':
    controller = CalibrationController(0.00327, 0.00585, 0.0037, 0.210, 0.297, -0.8, -0.8, 0.45)
    controller.calculate_postions()
    controller.publish_camera_frame()
    rospy.sleep(2)
    controller.get_world_pose_for_camera_pose(controller.close_positions[0][0], controller.close_positions[0][1],
                                              controller.close_positions[0][2])
