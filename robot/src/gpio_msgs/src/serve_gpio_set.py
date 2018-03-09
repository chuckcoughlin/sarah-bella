#!/usr/bin/env python
#
# Set the GPIO state of a specified pin. Return the pin with its new value.
# Package: gpio_msgs. Support for setting a single Raspberry Pi GPIO output.
#
import sys
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String
from gpio_msgs.srv import GPIOSet

	
def set_GPIO(request):
	response = GPIOSetResponse()
	response.channel = request.channel
	# CheckMode
	if GPIO.function(channel)=="IN":
		GPIO.output(channel,request.value)
		response.value = GPIO.input(response.channel)
		response.msg="Success"
	else:
		response.msg="GPIOSet error: channel ",channel," not configured as an IN"
		response.value = False
	return response


rospy.init_node('sb_serve_gpio_set')
serve = rospy.Service('/sb_serve_gpio_set',gpio_msgs.srv.GPIOSet,set_GPIO)
rospy.spin()
