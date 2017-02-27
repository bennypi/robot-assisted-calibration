#!/usr/bin/env python

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg

from std_msgs.msg import String


def publishCollisionObjects():
    print "Starting Node"
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub_co = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('CO_Publisher',
                    anonymous=True)


    pub.publish("hahaha")
    print "published"

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory, queue_size=100)

    addCollisionObject(pub_co, scene, group)


def addCollisionObject(pub_co, scene, group):
    ## Adding/Removing Objects and Attaching/Detaching Objects
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will define the collision object message
    print "============ Collision Objects"
    # collision_object = moveit_msgs.msg.CollisionObject()
    # collision_object.header.frame_id = group.get_planning_frame()
    # collision_object.id = "testbox"
    # primitive = shape_msgs.msg.SolidPrimitive
    # primitive.type = primitive.BOX
    # primitive.dimensions[0] = 2.0
    # primitive.dimensions[1] = 2.0
    # primitive.dimensions[2] = -0.2

    box_pose = geometry_msgs.msg.PoseStamped()
    header = std_msgs.msg.Header()
    header.frame_id = group.get_planning_frame()
    position = geometry_msgs.msg.Point()
    orientation = geometry_msgs.msg.Quaternion()
    position.x = 0.0
    position.y = 0.0
    position.z = -0.2
    orientation.w = 1.0
    box_pose.pose.position = position
    box_pose.pose.orientation = orientation

    co = moveit_msgs.msg.CollisionObject()
    co.operation = moveit_msgs.msg.CollisionObject.ADD
    co.id = "asdfa"
    co.header = header
    box = shape_msgs.msg.SolidPrimitive()
    box.type = shape_msgs.msg.SolidPrimitive.BOX
    box.dimensions = [2, 2, 0.2]
    co.primitives = [box]
    co.primitive_poses = [box_pose.pose]

    pub_co.publish(co)

    print("Adding collision object")
    # scene.add_box("onthefly", box_pose, (2, 2, 0.2))
    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    ## END_TUTORIAL
    print "============ STOPPING"


if __name__ == '__main__':
    try:
        publishCollisionObjects()
    except rospy.ROSInterruptException:
        pass

