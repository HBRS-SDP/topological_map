cmake_minimum_required(VERSION 2.8.3)
project(topological_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
  genmsg
  ropod_ros_msgs
  actionlib_msgs
  actionlib
)

catkin_python_setup()
## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

catkin_package(
  CATKIN_DEPENDS
  rospy
  ropod_ros_msgs
  actionlib_msgs
)

install(PROGRAMS
  scripts/topological_dispatcher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
