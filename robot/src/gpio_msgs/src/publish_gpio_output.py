#!/usr/bin/env python
#
# Publish the state of a single GPIO output that has changed since last
#         time it has been read. The poll rate is rapid, but we only report
#		  changes.
# Package: gpio_msgs. Support for publishing a single Raspberry Pi GPIO output.
#
from subprocess import call
import rospy
import sys
import RPi.GPIO as GPIO
from std_msgs.msg import String
import GPIOConfiguration
from gpio_msgs.msg import GPIOState
from gpio_msgs.msg import GPIOPin

	
pub = rospy.Publisher('/gpio_msgs',GPIOPin,queue_size=1)
rospy.init_node('sb_publish_gpio_output')
rate = rospy.Rate(10)  # 10 hz publish rate
# Specify all the args whether we use them or not.
msg = GPIOPin('label','channel','value','mode')

# Initialize the pin object fields, including values of outputs
pins = GPIOConfiguration.initialize(GPIOState())
for pin in pins:
	if pin.mode == "OUT":
		pin.value = GPIO.input(pin.channel)

while not rospy.is_shutdown():
	for pin in pins:
		if pin.mode=="OUT":
			val = GPIO.input(pin.channel)
			if val!=pin.value:
				pin.value = val
				pub.publish(pin)

	rate.sleep()

print "gpio_msgs.publish_gpio_output: complete"
