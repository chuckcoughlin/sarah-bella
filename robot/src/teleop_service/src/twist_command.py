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
from teleop_service.srv import TwistCommand,TwistCommandRequest,TwistCommandResponse

pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

def handleTwist(request):
	response = TwistCommandResponse()
	twist = Twist()
	twist.linear.x = request.linear_x
	twist.linear.y = request.linear_y
	twist.linear.z = request.linear_z
	twist.angular.x= request.angular_x
	twist.angular.y= request.angular_y
	twist.angular.z= request.angular_z
	response.msg = ""
	rospy.loginfo("Twist Command: %2.2f,%2.2f,%2.2f %2.2f,%2.2f,%2.2f"%(twist.linear.x,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z) )
	# Publish Twist
	pub.publish(twist)
	return response


if __name__ == "__main__":
	rospy.init_node('sb_twist_command')
	serve = rospy.Service('/sb_serve_twist_command',TwistCommand,handleTwist)
	rospy.spin()
