#!/usr/bin/env python

import sys
import rospy
import numpy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg
import tf.transformations


def init_publisher():
    global publisher
    publisher = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)
    rospy.init_node('CO_Publisher', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    global group
    group = moveit_commander.MoveGroupCommander("manipulator")
    global scene
    scene = moveit_commander.PlanningSceneInterface()
    global planning_frame
    planning_frame = group.get_planning_frame()


def create_header(planning_frame):
    co = moveit_msgs.msg.CollisionObject()
    header = std_msgs.msg.Header()
    header.frame_id = planning_frame
    co.header = header
    return co


def check_existing_id(id_string):
    if id_string in scene.get_known_object_names():
        return True
    else:
        return False


def remove_collision_object(id_string):
    if not check_existing_id(id_string):
        rospy.loginfo('ID %s is unknown in this planning scene', id_string)
        return

    co = create_header(planning_frame)
    co.id = id_string
    co.operation = 1

    publisher.publish(co)
    rospy.sleep(1)

    count = 0
    while check_existing_id(id_string):
        if count == 4:
            rospy.loginfo('Could not remove %s after five tries, aborting.', id_string)
            return
        rospy.loginfo('Could not remove collision object, will try again...')
        publisher.publish(co)
        rospy.sleep(1)
        count += 1


def add_collision_object(pos_x, pos_y, pos_z, box_x, box_y, box_z, id_string):
    if check_existing_id(id_string):
        rospy.loginfo('ID %s is already used in this planning scene', id_string)
        return

    box_pose = geometry_msgs.msg.PoseStamped()
    position = geometry_msgs.msg.Point()
    orientation = geometry_msgs.msg.Quaternion()
    position.x = pos_x
    position.y = pos_y
    position.z = pos_z
    orientation.w = 1.0
    box_pose.pose.position = position
    box_pose.pose.orientation = orientation

    co = create_header(planning_frame)
    co.id = id_string
    box = shape_msgs.msg.SolidPrimitive()
    box.type = shape_msgs.msg.SolidPrimitive.BOX
    box.dimensions = [box_x, box_y, box_z]
    co.primitives = [box]
    co.primitive_poses = [box_pose.pose]
    co.operation = 0

    publisher.publish(co)

    count = 1
    while not check_existing_id(id_string):
        if count == 5:
            rospy.loginfo('Could not add %s after five tries, aborting.', id_string)
            return
        rospy.loginfo('Could not add %s on attempt %d, retrying.', id_string, count)
        publisher.publish(co)
        rospy.sleep(1)
        count += 1
    rospy.loginfo('Collision object %s added.', id_string)


def add_calib_object_to_eef():
    id_string = 'calib_object'
    rospy.loginfo('Adding calibration plate to endeffector')
    # 210x297 size of DinA4
    # 'ee_link'

    rotation = tf.transformations.quaternion_from_euler(math.radians(90), math.radians(0), math.radians(0), 'syzx')

    box_pose = geometry_msgs.msg.PoseStamped()
    # offsets for caltab
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0.
    box_pose.pose.position.z = 0.
    box_pose.pose.orientation = geometry_msgs.msg.Quaternion(*rotation)

    co = create_header('ee_link')
    co.id = id_string
    box = shape_msgs.msg.SolidPrimitive()
    box.type = shape_msgs.msg.SolidPrimitive.BOX
    #  size of DinA4 plus extra space for attachment
    box.dimensions = [0.25, 0.37, 0.01]
    co.primitives = [box]
    co.primitive_poses = [box_pose.pose]
    co.operation = 0

    publisher.publish(co)

    count = 1
    while not check_existing_id(id_string):
        if count == 5:
            rospy.loginfo('Could not add %s after five tries, aborting.', id_string)
            return
        rospy.loginfo('Could not add %s on attempt %d, retrying.', id_string, count)
        publisher.publish(co)
        rospy.sleep(1)
        count += 1
    rospy.loginfo('Collision object %s added.', id_string)

    group.attach_object('calib_object', touch_links=['wrist_3_link', 'ee_link'])


def add_objects():
    add_collision_object(0.0, 0.0, -0.11, 2, 2, 0.2, 'table')
    add_collision_object(0, 0.7, 0.5, 2, .2, 1, 'left_wall')
    add_collision_object(0.6, 0, 0.5, .2, 2, 1, 'rear_wall')
    add_collision_object(0.4, -0.8, 0.5, .4, .4, 1, 'right_wall')
    add_collision_object(-0.8, -0.8, 0.45, .1, .1, .1, 'camera')


def remove_objects():
    remove_collision_object('table')
    remove_collision_object('left_wall')
    remove_collision_object('rear_wall')
    remove_collision_object('right_wall')
    remove_collision_object('camera')


if __name__ == '__main__':
    try:
        init_publisher()
    except rospy.ROSInterruptException:
        pass

    remove_collision_object('calib_object')
    add_calib_object_to_eef()
    remove_objects()
    add_objects()
    # if len(sys.argv) == 2 and sys.argv[1] == 'delete':
    #     rospy.loginfo('Removing Objects')
    #     remove_objects()
    # else:
    #     rospy.loginfo('Adding Collision Objects')
    #     add_objects()

    # Wand in +y: 0.8
    # Wand in +x: 0.7
    # Wand in -y: 0.7
    # Einbuchtung von +x 0.7 bis +x 0.3