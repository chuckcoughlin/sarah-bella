#!/usr/bin/env python
#
# Command the robot to change its current vector. The reason that we use
# a service instead of simply publishing a twist message is that the
# remote always starts after the robot is running. The subscriber cannot 
# be started before the publisher.
#
# We only care about the straight ahead velocity (x) and rotation on the plane (z).
# Updates are nominally every 100ms, Log every 2 seconds.
#
# We always respond to requests with no regard to the current behavior. If the 
# robot is given a behavior, like say "park", then it is up to the controller
# to suppress joystick commands (if appropriate). Likewise, we leave it to the
# tablet controller to pay attention to obstacle alerts.
#
# Package: teleop. Define the Twist message as a command.
#
import sys
import rospy
from geometry_msgs.msg import Twist
from teleop_service.srv import TwistCommand,TwistCommandRequest,TwistCommandResponse

REPORT_INTERVAL = 20
count = 0
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

def handleTwist(request):
	global count
	response = TwistCommandResponse()
	twist = Twist()
	twist.linear.x = -request.linear_x
	twist.linear.y = -request.linear_y
	twist.linear.z = -request.linear_z
	twist.angular.x= -request.angular_x
	twist.angular.y= -request.angular_y
	twist.angular.z= -request.angular_z
	response.msg = ""
	# Publish Twist
	pub.publish(twist)
	count = count+1
	if count%REPORT_INTERVAL==0:
		rospy.loginfo("Twist Command: %3.2f,%3.2f",twist.linear.x,twist.angular.z)
	return response


if __name__ == "__main__":
	rospy.init_node('sb_twist_command')
	serve = rospy.Service('/sb_serve_twist_command',TwistCommand,handleTwist)
	rospy.spin()
