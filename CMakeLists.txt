cmake_minimum_required(VERSION 3.0.2)
project(barlas)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
  cv_bridge
  image_transport
)

## Python setup
catkin_python_setup()

## Declare ROS messages, services and actions
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs mavros_msgs cv_bridge image_transport
)

## Build
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install scripts
catkin_install_python(PROGRAMS
  src/barlas_ros_bridge.py
  src/barlas_hibrid_controller.py
  src/mission_controller.py
  src/dart_recognition.py
  src/obstacle_avoidance.py
  src/motor_driver.py
  src/pid_controller.py
  src/pan_tilt_controller.py
  src/lidar_node.py
  src/ultrasonic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)