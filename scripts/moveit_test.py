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


def move_group_python_interface_tutorial():
    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    pub_co = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander("manipulator")

    print(scene.get_known_object_names())
    co = moveit_msgs.msg.CollisionObject()
    co.operation = moveit_msgs.msg.CollisionObject.REMOVE
    # co.id = 'asdfa'
    rate = rospy.Rate(10)
    while len(scene.get_known_object_names()) != 0:
        pub_co.publish(co)
        print(scene.get_known_object_names())
        rate.sleep()




if __name__ == '__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass

