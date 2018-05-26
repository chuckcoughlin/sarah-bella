#!/usr/bin/env python
#
# Set the GPIO state of a specified pin. Return the pin with its new value.
# Package: gpio_msgs. Support for setting a single Raspberry Pi GPIO output.
#
import sys
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String
from gpio_msgs.srv import GPIOPort,GPIOPortRequest,GPIOPortResponse
import GPIOConfiguration

# Get the pin configuraiions
PIN_COUNT = 40
GPIOConfiguration.configure()
pins = GPIOConfiguration.definePins()

def set_GPIO(request):
	response = GPIOPortResponse()
	channel = request.channel
	response.label = ""
	response.channel = channel
	# CheckMode
	mode = GPIOConfiguration.getMode(channel)
	response.mode = mode
	if mode=="OUT":
		GPIO.output(channel,request.value)
		try:
			response.value = GPIO.input(channel)
			response.msg="Success"
			rospy.loginfo("GPIO set: Pin %d %s => %s"%(channel,str(request.value),str(response.value)))
		except:
			response.msg = "GPIO set: configuration error: channel %d"%(channel)
			rospy.logwarn(response.msg)
			response.mode = "BAD"
	else:
		response.msg="GPIO set error: channel ",channel," not configured as an OUT"
		response.value = False
		rospy.loginfo(response.msg)
	return response


if __name__ == "__main__":
	rospy.init_node('sb_serve_gpio_set')
	serve = rospy.Service('/sb_serve_gpio_set',GPIOPort,set_GPIO)
	rospy.spin()
