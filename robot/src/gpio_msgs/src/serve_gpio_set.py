#!/usr/bin/env python
#
# Set the GPIO state of a specified pin. Return the pin with its new value.
# Package: gpio_msgs. Support for setting a single Raspberry Pi GPIO output.
#
import sys
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String
from gpio_msgs.srv import GPIOPort
import GPIOConfiguration

# Get the pin configuraiions
PIN_COUNT = 40
pins = GPIOConfiguration.definePins()

def set_GPIO(request):
	response = GPIOPortResponse()
	channel = request.channel
	response.label = ""
	response.channel = channel
	rospy.loginfo("sb_serve_gpio_set: Pin %d"%(channel))
	# CheckMode
	mode = str(GPIO.function(channel))
	if mode=="OUT":
		GPIO.output(channel,request.value)
		response.mode = mode
		response.value = request.value
		response.msg="Success"
	else:
		response.msg="GPIOSet error: channel ",channel," not configured as an OUT"
		response.mode = mode
		response.value = False
	rospy.loginfo(response.msg)
	return response


if __name__ == "__main__":
	rospy.init_node('sb_serve_gpio_set')
	serve = rospy.Service('/sb_serve_gpio_set',GPIOPort,set_GPIO)
	rospy.spin()
