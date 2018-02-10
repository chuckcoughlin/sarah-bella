cmake_minimum_required(VERSION 2.8.3)
project(sensor_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  message_generation 
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  BatteryState.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy 
  message_runtime 
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
