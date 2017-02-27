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
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    group = moveit_commander.MoveGroupCommander("manipulator")

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory, queue_size=100)

    # Moving Home
    # group.set_named_target('home')
    # group.plan()
    # group.go(wait=True)

    # moveArm(group)

    addCollisionObject(group, scene)


def moveArm(group):
    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector
    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.5
    pose_target.position.y = 0.5
    pose_target.position.z = 0.5
    group.set_pose_target(pose_target)
    ## Now, we call the planner to compute the plan
    ## and visualize it if successful
    ## Note that we are just planning, not asking move_group
    ## to actually move the robot
    plan1 = group.plan()
    group.go(wait=True)
    print "============ Waiting while moving"
    ## Planning to a joint-space goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## Let's set a joint space goal and move towards it.
    ## First, we will clear the pose target we had just set.
    group.clear_pose_targets()
    ## Then, we will get the current set of joint values for the group
    group_variable_values = group.get_current_joint_values()
    print "============ Joint values: ", group_variable_values
    ## Now, let's modify one of the joints, plan to the new joint
    ## space goal and visualize the plan
    group_variable_values[0] = 1.0
    group.set_joint_value_target(group_variable_values)
    plan2 = group.plan()
    group.go(wait=True)
    print "============ Waiting while RVIZ displays plan2..."


def addCollisionObject(group, scene):
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
    position = geometry_msgs.msg.Point()
    orientation = geometry_msgs.msg.Quaternion()
    position.x = 1.0
    position.y = 1.0
    position.z = -0.2
    orientation.w = 1.0
    box_pose.pose.position = position
    box_pose.pose.orientation = orientation
    box_pose.header = header

    co = moveit_msgs.msg.CollisionObject()
    co.operation = moveit_msgs.msg.CollisionObject.ADD
    co.id = "asdfa"
    co.header = header
    box = shape_msgs.msg.SolidPrimitive()
    box.type = shape_msgs.msg.SolidPrimitive.BOX
    box.dimensions = [2, 2, 0.2]
    co.primitives = [box]
    co.primitive_poses = [box_pose.pose]

    pub_co = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)
    pub_co.publish(co)
    buf = []
    co.serialize(buf)
    print(buf)

    print("Adding collision object")
    scene.add_box("onthefly", box_pose, (2, 2, 0.2))
    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    ## END_TUTORIAL
    print "============ STOPPING"


if __name__ == '__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass

