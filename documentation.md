# Documentations
## Installing
### INDIGO
- packages to install (apart from ros-indigo-desktop-full):`ros-indigo-ur-gazebo ros-indigo-ur5-moveit-config ros-indigo-ur-kinematics ros-indigo-moveit-simple-controller-manager ros-indigo-moveit`

### KINETIC
- install (apart from ros-kinetic-desktop-full): `ros-kinetic-moveit-simple-controller-manager ros-kinetic-moveit`
- clone `https://github.com/ros-industrial/universal_robot` into your catkin_ws/src 
- add the following lines to `ur_kinematics/CMakeLists.txt` after `catkin_package(...)`

    ```
    # If the compiler supports C++11, enable the flag for it. This is needed
    # on kinetic and newer because some headers that moveit headers include
    # use C++11 features.
    #
    # If the compiler doesn't support C++11, then we hope that we're on an older
    # rosdistro and we don't need it.
    #
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    if(COMPILER_SUPPORTS_CXX11)
        add_compile_options(-std=c++11)
    endif()
    ```
- run `catkin build`
- now install `ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control`

## Running
### Simulation with gazebo
1. ```roslaunch ur_gazebo ur5.launch```
2. ```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true```
3. ```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```

### Add the table as a collision object

```rosrun robot_assisted_calibration publish_collision_objects.py```
