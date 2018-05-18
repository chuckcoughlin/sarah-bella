#!/usr/bin/env python
#
# Command the robot to change its current vector. The reason that we use
# a service instead of simply publishing a twist message is that the
# remote always starts after the robot is running. The subscriber cannot 
# be started before the publisher.
#
# Package: teleop. Define the Twist message as a command.
#
import sys
import rospy
from geometry_msgs.msg import Twist
from teleop.srv import TwistCommand,TwistCommandRequest,TwistCommandResponse

pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

def handleTwist(request):
	response = TwistCommandResponse()
	twist = Twist()
	twist.linear.x = request.linear_x
	twist.linear.y = request.linear_y
	twist.linear.z = request.linear_z
	twist.angular.x= requerst.angular_x
	twist.angular.y= requerst.angular_y
	twist.angular.z= requerst.angular_z
	response.msg = ""
	rospy.loginfo("sb_serve_twist_command:" )
	# Publish Twist
	pub.publist(twist)
	return response


if __name__ == "__main__":
	rospy.init_node('sb_twist_command')
	serve = rospy.Service('/sb_serve_twist_command',TwistCommand,handleTwist)
	rospy.spin()
