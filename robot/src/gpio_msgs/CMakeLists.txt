cmake_minimum_required(VERSION 2.8.3)
project(gpio_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  message_generation 
  std_msgs
)

add_service_files(
  DIRECTORY srv
  FILES
  GPIOSet.srv
)

add_message_files(
  DIRECTORY msg
  FILES
  GPIOPin.msg
  GPIOState.msg
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
