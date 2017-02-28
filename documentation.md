# Documentations
## Installing
- packages to install (apart from ros-indigo-desktop-full): ```ros-indigo-ur-gazebo ros-indigo-ur5-moveit-config ros-indigo-ur-kinematics ros-indigo-moveit-simple-controller-manager```

## Running
### Simulation with gazebo
1. ```roslaunch ur_gazebo ur5.launch```
2. ```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true```
3. ```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```

### Add the table as a collision object

```rosrun robot_assisted_calibration publish_collision_objects.py```
