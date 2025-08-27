cmake_minimum_required(VERSION 3.0.2)
project(barlas)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros
  mavros_msgs
  gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    mavros
    mavros_msgs
    gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Launch dosyalarını kopyala
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# URDF dosyalarını kopyala
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
)

# World dosyalarını kopyala
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  FILES_MATCHING PATTERN "*.world"
)
