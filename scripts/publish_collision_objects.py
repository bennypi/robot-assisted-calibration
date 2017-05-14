#!/usr/bin/env python
import sys
from Crypto.PublicKey.pubkey import pubkey

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg


def init_publisher():
    global publisher
    publisher = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)
    rospy.init_node('CO_Publisher', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
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
        print 'ID', id_string, 'is unknown in this planning scene'
        return

    co = create_header(planning_frame)
    co.id = id_string
    co.operation = 1

    publisher.publish(co)
    rospy.sleep(1)

    count = 0
    while check_existing_id(id_string):
        if count == 4:
            print 'Could not remove', id_string, 'after five tries, aborting.'
            return
        print 'Could not remove collision object, will try again...'
        publisher.publish(co)
        rospy.sleep(1)
        count += 1


def add_collision_object(pos_x, pos_y, pos_z, box_x, box_y, box_z, id_string):
    if check_existing_id(id_string):
        print 'ID', id_string, 'is already used in this planning scene'
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

    count = 0
    while not check_existing_id(id_string):
        if count == 4:
            print 'Could not add', id_string, 'after five tries, aborting.'
            return
        print 'Could not add collision object, will try again...'
        publisher.publish(co)
        rospy.sleep(1)
        count += 1


def add_objects():
    add_collision_object(0.0, 0.0, -0.2, 2, 2, 0.2, 'table')
    add_collision_object(1.0, 0.0, 0.5, 0.01, 0.295, 0.21, 'chessboard')


def remove_objects():
    remove_collision_object('table')
    remove_collision_object('chessboard')


if __name__ == '__main__':
    try:
        init_publisher()
    except rospy.ROSInterruptException:
        pass

    if len(sys.argv) == 2 and sys.argv[1] == 'delete':
        print 'Removing Objects'
        remove_objects()
    else:
        print 'Adding Collision Objects'
        add_objects()
