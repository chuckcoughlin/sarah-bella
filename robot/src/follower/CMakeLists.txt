cmake_minimum_required(VERSION 2.8.3)
project(follower)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  turtlebot3_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

catkin_package(
  rospy
  geometry_msgs
  sensor_msgs
  message_runtime
  std_msgs
)
