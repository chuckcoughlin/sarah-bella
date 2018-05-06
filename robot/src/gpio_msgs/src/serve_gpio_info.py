#!/usr/bin/env python
#
# Get the GPIO mode of a specified pin. Set the value true and set the msg as the mode,label.
#     On error, set the value false and the message is the error string.
# Package: gpio_msgs. 
#
import sys
import rospy
from std_msgs.msg import String
from gpio_msgs.msg import GPIOPin
from gpio_msgs.msg import GPIOState
from gpio_msgs.srv import GPIOPort
import GPIOConfiguration

PIN_COUNT = 40
pins = GPIOConfiguration.definePins()

def config_GPIO(request):
	response = GPIOSetResponse()
	channel = request.channel
	response.channel = channel
	if channel>0 and channel<= PIN_COUNT:
		pin = pins[channel]
		response.value = True
		response.label = pin.label
		response.mode  = pin.mode
		response.msg="Success"
	else:
		response.msg="GPIOInfo error: channel ",channel," is out-of-range"
		response.value = False
	rospy.loginfo(response.msg)
	return response


if __name__ == "__main__":
	rospy.init_node('sb_serve_gpio_set')
	serve = rospy.Service('/sb_serve_gpio_info',GPIOPort,config_GPIO)
	rospy.spin()
