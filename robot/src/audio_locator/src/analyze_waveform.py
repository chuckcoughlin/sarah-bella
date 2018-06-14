#!/usr/bin/env python
#
# Analyze the stereo waveform and determine a direction for the
# robot. Note this only happens if the behavior is set to "come".
# Command the robot to change its current vector toward the audio source.
# Publish the waveform for analysis on the tablet.
# Updates are nominally every 100ms, Log every 2 seconds.
#
# Package: audio_locator
#
import sys
import rospy
from geometry_msgs.msg import Twist
from teleop_service.srv import TwistCommand,TwistCommandRequest,TwistCommandResponse

REPORT_INTERVAL = 20
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
count = 0

# Post a twist request to move/turn the robot toward the sound source
def handleSignal(request):
	response = TwistCommandResponse()
	twist = Twist()
	twist.linear.x = request.linear_x
	twist.linear.y = request.linear_y
	twist.linear.z = request.linear_z
	twist.angular.x= request.angular_x
	twist.angular.y= request.angular_y
	twist.angular.z= request.angular_z
	response.msg = ""
	# Publish Twist
	pub.publish(twist)
	count = count+1
	if count%REPORT_INTERVAL==0:
		rospy.loginfo("Twist Command: %3.2f,%3.2f",twist.linear.x,twist.angular.z) )
	return response


if __name__ == "__main__":
	rospy.init_node('sb_serve_analyze_audio')
	serve = rospy.Service('/sb_serve_analyze_audio',SignalAudio,handleSignal)
	rospy.spin()
