cmake_minimum_required(VERSION 2.8.3)
project(follower)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  teleop_service
)

catkin_package( CATKIN_DEPENDS
  rospy
  geometry_msgs
  sensor_msgs
  teleop_service
  message_runtime
)
