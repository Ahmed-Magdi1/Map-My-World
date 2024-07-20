cmake_minimum_required(VERSION 3.0.2)
project(my_robot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  gazebo_ros
  sensor_msgs
  message_generation
  urdf
  xacro
  nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

