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


def add_collision_object():
    pub_co = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)
    rospy.init_node('CO_Publisher', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander("manipulator")
    scene = moveit_commander.PlanningSceneInterface()
    planning_frame = group.get_planning_frame()

    box_pose = geometry_msgs.msg.PoseStamped()
    position = geometry_msgs.msg.Point()
    orientation = geometry_msgs.msg.Quaternion()
    position.x = 0.0
    position.y = 0.0
    position.z = -0.2
    orientation.w = 1.0
    box_pose.pose.position = position
    box_pose.pose.orientation = orientation

    co = create_header(planning_frame)
    co.id = "asdfa"
    box = shape_msgs.msg.SolidPrimitive()
    box.type = shape_msgs.msg.SolidPrimitive.BOX
    box.dimensions = [2, 2, 0.2]
    co.primitives = [box]
    co.primitive_poses = [box_pose.pose]

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
    try:
        add_collision_object()
    except rospy.ROSInterruptException:
        pass
