cmake_minimum_required(VERSION 2.8.3)
project(gpio_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  message_generation 
)

add_service_files(
  DIRECTORY srv
  FILES
  GPIOPort.srv
)

add_message_files(
  DIRECTORY msg
  FILES
  GPIOPin.msg
  GPIOState.msg
)

generate_messages(
  DEPENDENCIES 
)

catkin_package(
  CATKIN_DEPENDS
  rospy 
  message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
