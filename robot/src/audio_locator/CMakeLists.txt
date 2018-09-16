cmake_minimum_required(VERSION 2.8.3)
project(audio_locator)

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
    AudioLocation.msg
    StereoWaveform.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    SignalAudio.srv
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
