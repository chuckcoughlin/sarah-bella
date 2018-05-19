#!/usr/bin/env python
#
# Get the GPIO state of a specified pin. Return the pin with its new value.
# Package: gpio_msgs. Support for getting a single Raspberry Pi GPIO input.
#
import sys
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String
from gpio_msgs.srv import GPIOPort,GPIOPortRequest,GPIOPortResponse
import GPIOConfiguration

GPIOConfiguration.configure()
# Read the pin configurations
PIN_COUNT = 40
pins = GPIOConfiguration.definePins()

def str2bool(s):
	return s.lower() in ('true','t','1','yes')

def get_GPIO(request):
	response = GPIOPortResponse()
	channel = request.channel
	response.label = ""
	response.channel = request.channel
	# CheckMode
	mode = GPIOConfiguration.getMode(channel)
	response.mode = mode
	if mode=="IN" or mode=="OUT":
		try:
			response.value = GPIO.input(channel)
			response.msg="Success"
			rospy.loginfo("Get channel %d = %s"%(channel,str(response.value)))
		except:
			response.msg = "GPIO Get Configuration error: channel %d"%(channel)
			response.mode="BAD"
			rospy.logwarn(response.msg)
	else:
		response.msg="GPIO Get error: channel ",channel," not configured as an IN or OUT"
		response.value = False
		rospy.loginfo(response.msg)
	return response


if __name__ == "__main__":
	rospy.init_node('sb_serve_gpio_get')
	serve = rospy.Service('/sb_serve_gpio_get',GPIOPort,get_GPIO)
	rospy.spin()
