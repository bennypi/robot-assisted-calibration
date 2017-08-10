cmake_minimum_required(VERSION 2.8.3)
project(calibration_perception)

## Make sure HALCONROOT is set
if(NOT DEFINED ENV{HALCONROOT})
  message( FATAL_ERROR "PLEASE SET HALCONROOT" )
endif()

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs image_transport actionlib actionlib_msgs calibration_executive)
find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

#add_action_files(
#  DIRECTORY action
#  FILES FindCaltab.action
#  FILES Calibrate.action
#)

#generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include $ENV{HALCONROOT}/include $ENV{HALCONROOT}/include/halconcpp
  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs actionlib_msgs calibration_executive
)

include_directories(include $ENV{HALCONROOT}/include $ENV{HALCONROOT}/include/halconcpp
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

link_directories($ENV{HALCONROOT}/lib/$ENV{HALCONARCH})

# calibration_perception_node
add_executable(${PROJECT_NAME}_node src/caltab_detector_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   halconcpp
   halcon
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_alexis_example src/alexis_example.cpp)
add_dependencies(${PROJECT_NAME}_alexis_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_alexis_example
        halconcpp
        halcon
        X11
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )
