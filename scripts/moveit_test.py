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
        moveit_msgs.msg.DisplayTrajectory)

    # Moving Home
    group.set_named_target('home')
    group.plan()
    group.go(wait=True)

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


    ## Adding/Removing Objects and Attaching/Detaching Objects
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will define the collision object message
    print "============ Collision Objects"
    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id = group.get_planning_frame()

    collision_object.id = "testbox"

    primitive = shape_msgs.msg.SolidPrimitive
    primitive.type = primitive.BOX
    primitive.dimensions[0] = 2.0
    primitive.dimensions[1] = 2.0
    primitive.dimensions[2] = -0.2

    box_pose = geometry_msgs.msg.Pose
    position = geometry_msgs.msg.Point
    orientation = geometry_msgs.msg.Quaternion

    position.x = 1.0
    position.y = 1.0
    position.z = -0.2

    orientation.w = 1.0

    box_pose.position = position
    box_pose.orientation = orientation

    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(box_pose)
    collision_object.operation = collision_object.ADD

    collision_objects = []
    collision_objects.append(collision_object)

    

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

