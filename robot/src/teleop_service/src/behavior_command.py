#!/usr/bin/env python
#
# Command the robot to change its current application. These applications are
# all lidar-related.  We use a service instead of simply publishing a message 
# because the # remote always starts after the robot is running. The subscriber cannot 
# be started before the publisher.
#
# We always respond to requests with no regard to the current behavior. If the 
# robot is given a behavior, like say "park", then it is up to the controller
# to suppress joystick commands (if appropriate). Likewise, we leave it to the
# tablet controller to pay attention to obstacle alerts.
#
# Use this method to open the teleop status publisher as a guarantee that
# the topic will always exist. The various behaviors all make use of it.
#
# Package: teleop_service. Define the behavior.
#
import sys
import rospy
from teleop_service.msg import Behavior, TeleopStatus
from teleop_service.srv import BehaviorCommand,BehaviorCommandRequest,BehaviorCommandResponse

pub = rospy.Publisher('/sb_behavior',Behavior,queue_size=1)
spub = rospy.Publisher('sb_teleop_status',TeleopStatus,queue_size=1)

def handleBehavior(request):
	response = BehaviorCommandResponse()
	behavior = Behavior()
	behavior.state = request.behavior
	opstatus = TeleopStatus()
	opstatus.status=str(behavior.state)+' started ...'
	response.msg = ""
	# "joystick" means all control is external
	if behavior.state=="joystick":
		rospy.set_param("robot/reset","true")
	else:
		rospy.set_param("robot/reset","false")
	
	# Publish Behavior
	pub.publish(behavior)
	spub.publish(opstatus)
	rospy.loginfo("Behavior Command: New behavior is %s",behavior.state)
	return response


if __name__ == "__main__":
	rospy.init_node('sb_behavior_command')
	serve = rospy.Service('/sb_serve_behavior_command',BehaviorCommand,handleBehavior)
	rospy.spin()
