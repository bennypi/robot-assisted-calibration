#!/usr/bin/env python

import rospy
import math
import tf
import tf_conversions
import geometry_msgs.msg
import MovementController
import actionlib

from threading import Thread
from caltab_detector.msg import CalibrateAction, CalibrateGoal
from visualization_msgs.msg import Marker, MarkerArray


class HighLevelExecutive(object):
    """
    This class is the High Level Executive for the robot assisted calibration.
    
    This class will determine the different positions for the calibration object and will pass these 
    to the MovementController. After every position has been executed, the calibrate action will be called.
    """
    def __init__(self, sensor_height, sensor_width, focal_length, caltab_height, caltab_width, camera_x, camera_y,
                 camera_z):
        self.t = Thread(target=self.thread_publisher)
        rospy.init_node('calibration_controller')
        rospy.loginfo('Starting CalibrationController')

        self.close_distance_factor = rospy.get_param('/calibration_controller/close_distance_factor', 0.8)
        self.medium_distance_factor = rospy.get_param('/calibration_controller/medium_distance_factor', 0.5)
        self.far_distance_factor = rospy.get_param('/calibration_controller/far_distance_factor', 0.3)
        rospy.loginfo(
            'Setting distance factors to {}, {} and {}'.format(self.close_distance_factor, self.medium_distance_factor,
                                                               self.far_distance_factor))

        self.keep_thread_running = True
        self.robot_reachable_distance = 0.9
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

        rospy.loginfo('Trying to connect to the calibrate action server')
        self.calibrate_client = actionlib.SimpleActionClient('Calibrate', CalibrateAction)
        self.calibrate_client.wait_for_server()

        self.marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100)
        self.marker_list = MarkerArray()

        rospy.loginfo('Finished initialization of CalibrationController')

    def check_positions_in_range(self):
        """
        This function will check for every position if the distance to the position is less than the 
        robot_reachable_distance. It will return the ratio.
        
        :return: The ratio of the reachable positions
        :rtype: float
        """
        reachable = 0
        total = 0
        reachable, total = self.check_positions_in_range_for_list(reachable, total, self.close_positions_world)
        reachable, total = self.check_positions_in_range_for_list(reachable, total, self.medium_positions_world)
        reachable, total = self.check_positions_in_range_for_list(reachable, total, self.far_positions_world)

        return float(reachable) / float(total)

    def check_positions_in_range_for_list(self, reachable, total, list):
        """
        This function will check for every position on the list if the distance to this position is closer than the 
        robot_reachable_distance.
        
        :param reachable: Contains the number of reachable positions
        :type reachable: int
        :param total: Contains the number of checked positions 
        :type total: int
        :param list: The list with the positions that should be checked
        :type list: list of positions
        :return: reachable, total
        :rtype: int, int
        """
        for pose in list:
            total += 1
            distance_to_base = math.sqrt(pose[0] ** 2 + pose[1] ** 2 + pose[2] ** 2)
            if distance_to_base < self.robot_reachable_distance:
                reachable += 1
            else:
                rospy.logwarn('Position not in range: {}, distance to base: {}'.format(pose, distance_to_base))
        return reachable, total

    def calculate_distance(self, decrease_factor):
        """
        Calculate the distance from the caltab to the camera so that caltab_height = image_height * decrease_factor.
        
        :param decrease_factor: The factor that specifies how high the caltab should be
        :type decrease_factor: float
        :return: The distance between the caltab and camera
        :rtype: float
        """
        return self.caltab_height * self.focal_length / (self.sensor_height * decrease_factor)

    def fov_height_for_distance(self, distance):
        """
        Calculate the height of the field of view for the given distance.
        
        :param distance: The distance for the field of view
        :type distance: float
        :return: Height of the field of view
        :rtype: float
        """
        return self.sensor_height * distance / self.focal_length

    def fov_width_for_distance(self, distance):
        """
        Calculate the width of the field of view for the given distance.

        :param distance: The distance for the field of view
        :type distance: float
        :return: Width of the field of view
        :rtype: float
        """
        return self.sensor_width * distance / self.focal_length

    def calculate_all_distances(self):
        """
        Calculate the three different distances between the caltab and the camera for the three different 
        distance factors.
        """
        self.close_distance = self.calculate_distance(self.close_distance_factor)
        self.medium_distance = self.calculate_distance(self.medium_distance_factor)
        self.far_distance = self.calculate_distance(self.far_distance_factor)

    def calculate_postions_in_camera_frame(self):
        """
        Calculate the different positions for every of the three distances. Note that the positions are relative 
        to the camera frame.
        
        There is only one position for the close distance. The position will be stored in close_positions_camera.
        
        There are four positions for the medium distance. The positions will be stored in medium_positions_camera.
        
        There are nine positions for the far distance. The positions will be stored in far_positions_camera.
        """

        # Get the different distances
        self.calculate_all_distances()

        # The only position for this distance is centered in front of the camera
        self.close_positions_camera = [[self.close_distance, 0, 0]]

        # Calculate the dimensions of the field of view for the medium distance
        fov_height = self.fov_height_for_distance(self.medium_distance)
        fov_width = self.fov_width_for_distance(self.medium_distance)

        # Calculate the positions for the first row
        self.medium_positions_camera.append(
            [self.medium_distance, -(fov_width / 2 - self.caltab_width / 2), fov_height / 2 - self.caltab_height / 2])
        self.medium_positions_camera.append(
            [self.medium_distance, fov_width / 2 - self.caltab_width / 2, fov_height / 2 - self.caltab_height / 2])

        # Calculate the positions for the second row
        self.medium_positions_camera.append(
            [self.medium_distance, -(fov_width / 2 - self.caltab_width / 2),
             -(fov_height / 2 - self.caltab_height / 2)])
        self.medium_positions_camera.append(
            [self.medium_distance, fov_width / 2 - self.caltab_width / 2, -(fov_height / 2 - self.caltab_height / 2)])

        # Now get the dimensions of the field of view for the far distance
        fov_height = self.fov_height_for_distance(self.far_distance)
        fov_width = self.fov_width_for_distance(self.far_distance)

        # Calculate the positions for the first row
        self.far_positions_camera.append(
            [self.far_distance, -(fov_width / 2 - self.caltab_width / 2), fov_height / 2 - self.caltab_height / 2])
        self.far_positions_camera.append([self.far_distance, 0, fov_height / 2 - self.caltab_height / 2])
        self.far_positions_camera.append(
            [self.far_distance, fov_width / 2 - self.caltab_width / 2, fov_height / 2 - self.caltab_height / 2])

        # Calculate the positions for the second row
        self.far_positions_camera.append(
            [self.far_distance, -(fov_width / 2 - self.caltab_width / 2), 0])
        self.far_positions_camera.append([self.far_distance, 0, 0])
        self.far_positions_camera.append(
            [self.far_distance, fov_width / 2 - self.caltab_width / 2, 0])

        # Calculate the positions for the third row
        self.far_positions_camera.append(
            [self.far_distance, -(fov_width / 2 - self.caltab_width / 2), -(fov_height / 2 - self.caltab_height / 2)])
        self.far_positions_camera.append(
            [self.far_distance, 0, -(fov_height / 2 - self.caltab_height / 2)])
        self.far_positions_camera.append(
            [self.far_distance, fov_width / 2 - self.caltab_width / 2, -(fov_height / 2 - self.caltab_height / 2)])

    def transform_camera_pose_to_world_pose(self):
        """
        Use TF to transform the positions from the camera frame to the world frame so that they are usable by moveit.
        
        Iterate every list of positions and store the newly calculated positions in the positions_world lists.
        """
        for pose in self.close_positions_camera:
            self.close_positions_world.append(self.get_world_pose_for_camera_pose(pose))

        for pose in self.medium_positions_camera:
            self.medium_positions_world.append(self.get_world_pose_for_camera_pose(pose))

        for pose in self.far_positions_camera:
            self.far_positions_world.append(self.get_world_pose_for_camera_pose(pose))

    def get_world_pose_for_camera_pose(self, pose):
        """
        Transform a given position from the camera frame to the world frame using TF.
        
        :param pose: The position in the camera frame as (x,y,z)
        :type pose: list of float
        :return: The position in the world frame as (x,y,z)
        :rtype: list of float
        """

        # Create a point stamped from the given position
        camera_point = geometry_msgs.msg.PointStamped()
        camera_point.header.stamp = rospy.Time.now()
        camera_point.header.frame_id = 'camera'
        camera_point.point.x = pose[0]
        camera_point.point.y = pose[1]
        camera_point.point.z = pose[2]

        # Wait for the transformation to be available
        time = rospy.Time().now()
        self.listener.waitForTransform('camera', 'world', time, rospy.Duration(5))
        world_point = self.listener.transformPoint('world', camera_point)

        # Return the new coordinates
        return [world_point.point.x, world_point.point.y, world_point.point.z]

    def get_camera_orientation(self):
        """
        Calculate the orientation of the camera in the world frame. This is needed to use TF to transform 
        positions from the camera frame to the world frame
        """

        # Create the vector from the camera to the robot
        vector_x = self.robot_x - self.camera_x
        vector_y = self.robot_y - self.camera_y
        vector_z = self.robot_z - self.camera_z

        # Calculate yaw and pitch from this vector
        yaw = math.atan2(vector_y, vector_x)
        pitch = -math.asin(vector_z)

        # Create the quaternion from the euler angles
        self.quaternion = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0, pitch, yaw))

    def thread_publisher(self):
        """
        Publish the transformation from the camera frame to the world frame from a second thread.
        """

        # Publish the transformation to TF with a rate of 50 Hz
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and self.keep_thread_running is True:
            self.broadcaster.sendTransform((self.camera_x, self.camera_y, self.camera_z),
                                           (self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w),
                                           rospy.Time.now(),
                                           'camera', 'world')
            rate.sleep()

    def publish_camera_frame(self):
        """
        Calculate the camera orientation in the world frame, then create a thread to publish the transformation to TF.
        """
        executive.get_camera_orientation()
        self.t.start()
        # Wait for transformation to be published
        rospy.sleep(2)

    def create_marker_for_position(self, position):
        """
        For the given position, create a marker and append it to the marker_list.
        
        :param position: The position that should be marked as (x,y,z)
        :type position: list of float
        """

        marker = Marker()
        marker.header.frame_id = "/world"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        self.marker_list.markers.append(marker)

    def publish_markers(self):
        """
        Create a marker for every position that is calculated and publish these markers as a MarkerArray.
        """

        # Call the create_marker_for_position function for every position
        for position in executive.close_positions_world:
            self.create_marker_for_position(position)
        for position in executive.medium_positions_world:
            self.create_marker_for_position(position)
        for position in executive.far_positions_world:
            self.create_marker_for_position(position)

        # Renumber the marker IDs
        id = 0
        for m in self.marker_list.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        self.marker_publisher.publish(self.marker_list)

    def do_calibration(self):
        """
        Do the calibration by calling the calibrate action server.
        """
        goal = CalibrateGoal()
        goal.goal = 0
        self.calibrate_client.send_goal(goal)
        print 'goal sent'
        self.calibrate_client.wait_for_result()
        print self.calibrate_client.get_result()


if __name__ == '__main__':
    # Create an instance of this class
    executive = HighLevelExecutive(0.00327, 0.00585, 0.0037, 0.210, 0.297, -0.7, 0.7, 0.35)

    # Create an instance of the MovementController
    movement_controller = MovementController.MovementController("/MoveArm", "/FindCaltab")

    # Calculate the different positions for the calibration object and transform these to the world frame
    executive.calculate_postions_in_camera_frame()
    executive.publish_camera_frame()
    executive.transform_camera_pose_to_world_pose()
    executive.publish_markers()

    # Check that 75% of the calculated positions should be reachable
    in_range = executive.check_positions_in_range()
    if in_range < 0.75:
        rospy.logerr('Less than {:02.0f}% of the calculated positions seem to be in reach.'.format(in_range * 100))
        rospy.logerr('This can result in a bad calibration. Please check the saved Images and results.')

    # Move to the close position
    for position in executive.close_positions_world:
        movement_controller.execute_different_orientations(position)

    # Do the calibration
    executive.do_calibration()
    raw_input("Press Enter to continue...")

    # Move the medium positions
    for position in executive.medium_positions_world:
        movement_controller.execute_different_orientations(position)

    # Move to the far positions
    for position in executive.far_positions_world:
        movement_controller.execute_different_orientations(position)

    # Stop the thread
    executive.keep_thread_running = False
    executive.t.join()
