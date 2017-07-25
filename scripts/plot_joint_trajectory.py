#!/usr/bin/env python

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib.pyplot as plt


def callback(data):
    link_0_position = []
    link_1_position = []
    link_2_position = []
    link_3_position = []
    link_4_position = []
    link_5_position = []
    link_0_velocities = []
    link_1_velocities = []
    link_2_velocities = []
    link_3_velocities = []
    link_4_velocities = []
    link_5_velocities = []
    link_0_accelerations = []
    link_1_accelerations = []
    link_2_accelerations = []
    link_3_accelerations = []
    link_4_accelerations = []
    link_5_accelerations = []
    duration = []

    goal = data.goal
    assert isinstance(goal.trajectory.points, list)
    traj = goal.trajectory.points
    for p in traj:
        assert isinstance(p, JointTrajectoryPoint)
        link_0_position.append(p.positions[0])
        link_1_position.append(p.positions[1])
        link_2_position.append(p.positions[2])
        link_3_position.append(p.positions[3])
        link_4_position.append(p.positions[4])
        link_5_position.append(p.positions[5])
        link_0_velocities.append(p.velocities[0])
        link_1_velocities.append(p.velocities[1])
        link_2_velocities.append(p.velocities[2])
        link_3_velocities.append(p.velocities[3])
        link_4_velocities.append(p.velocities[4])
        link_5_velocities.append(p.velocities[5])
        link_0_accelerations.append(p.accelerations[0])
        link_1_accelerations.append(p.accelerations[1])
        link_2_accelerations.append(p.accelerations[2])
        link_3_accelerations.append(p.accelerations[3])
        link_4_accelerations.append(p.accelerations[4])
        link_5_accelerations.append(p.accelerations[5])
        duration.append(rospy.Time(p.time_from_start.secs, p.time_from_start.nsecs).to_time())


    rospy.loginfo(duration)
    rospy.loginfo(link_0_position)

    plt.subplot(221)
    plt.plot(duration, link_0_position,duration, link_1_position, duration, link_2_position, duration, link_3_position, duration, link_4_position, duration, link_5_position)
    plt.subplot(222)
    plt.plot(duration, link_0_velocities, duration, link_1_velocities, duration, link_2_velocities, duration, link_3_velocities,
             duration, link_4_velocities, duration, link_5_velocities)
    plt.subplot(223)
    plt.plot(duration, link_0_accelerations, duration, link_1_accelerations, duration, link_2_accelerations, duration,
             link_3_accelerations,
             duration, link_4_accelerations, duration, link_5_accelerations)
    plt.show()


if __name__ == '__main__':
    rospy.init_node('plotter')
    subscriber = rospy.Subscriber('/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, callback)
    rospy.loginfo('started')
    rospy.spin()