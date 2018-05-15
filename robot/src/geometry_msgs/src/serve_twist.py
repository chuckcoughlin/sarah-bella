#!/usr/bin/env python
#
# Command the robot to change its current vector. The reason that we use
# a service instead of simply listening for a twist message is that the
# remote always starts after the robot is running. The subscriber cannot 
# be started before the publisher.
#
# Package: geometry_msgs. Define the Twist message as a command.
#
import sys
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.srv import TwistCommand,TwistCommandRequest,TwistCommandResponse

pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

def handleTwist(request):
	response = TwistCommandResponse()
	twist = Twist()
	twist.linear = request.linear
	twist.angular= requerst.angular
	response.msg = ""
	rospy.loginfo("sb_serve_twist_command:" )
	# Publish Twist
	pub.publist(twist)
	return response


if __name__ == "__main__":
	rospy.init_node('sb_serve_twist~_command')
	serve = rospy.Service('/sb_serve_twist_command',TwistCommand,handleTwist)
	rospy.spin()
