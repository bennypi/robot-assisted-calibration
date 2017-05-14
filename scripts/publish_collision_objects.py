#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg


def create_header(planning_frame):
    co = moveit_msgs.msg.CollisionObject()
    header = std_msgs.msg.Header()
    header.frame_id = planning_frame
    co.header = header
    return co


def add_collision_object(pos_x, pos_y, pos_z, box_x, box_y, box_z, id_string, op):
    pub_co = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)
    rospy.init_node('CO_Publisher', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander("manipulator")
    scene = moveit_commander.PlanningSceneInterface()
    planning_frame = group.get_planning_frame()

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
    co.operation = op

    print (co)

    before = len(scene.get_known_object_names())
    pub_co.publish(co)
    print(scene.get_known_object_names())
    after = len(scene.get_known_object_names())
    if after > before:
        print ('Added collision object')
    else:
        print ('Error adding collision object')


if __name__ == '__main__':
    if len(sys.argv) == 2 and sys.argv[1] == 'delete':
        operation = 1
        print 'Removing Objects'
    else:
        operation = 0
        'Adding Collision Objects'

    try:
        add_collision_object(0.0, 0.0, -0.2, 2, 2, 0.2, 'table', operation)
        add_collision_object(1.0, 0.0, 0.5, 0.01, 0.295, 0.21, 'chessboard', operation)
    except rospy.ROSInterruptException:
        pass
