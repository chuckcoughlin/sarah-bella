cmake_minimum_required(VERSION 2.8.3)
project(teleop_service)

find_package(catkin 
  REQUIRED COMPONENTS 
    rospy
    geometry_msgs 
    sensor_msgs 
    message_generation 
)


add_message_files(
  DIRECTORY msg
  FILES
	Behavior.msg
    ObstacleDistance.msg
	TeleopStatus.msg
)

add_service_files(
  DIRECTORY srv
  FILES
	BehaviorCommand.srv
    TwistCommand.srv
)

generate_messages(
  DEPENDENCIES 
	std_msgs
    geometry_msgs
    sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS 
    rospy
    sensor_msgs
    geometry_msgs 
    message_runtime 
    std_msgs
)
