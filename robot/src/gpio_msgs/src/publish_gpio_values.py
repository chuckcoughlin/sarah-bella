#!/usr/bin/env python
#
# Publish the state of all changed GPIO outputs at once. At a configurable
# interval, publish them all.
# Package: gpio_msgs. Support for publishing Raspberry Pi GPIO outputs.
#                     On start of the method we configure the GPIO pins.
#
# NOTE: Make sure that different message types get different topics, else
#       "ros could not process inbound connection: topic types do not match".
#       
#		We normally only report changes. However if "/gpio_msgs/publish_all"
#       is set, we would like to publish the entire array and reset parameter.
#       We assume the list of all outputs meets the 1024 message size limit.
#
from subprocess import call
import rospy
import sys
import RPi.GPIO as GPIO
from std_msgs.msg import String
from gpio_msgs.msg import GPIOState
from gpio_msgs.msg import GPIOPin
import GPIOConfiguration

# Configure the GPIO pins.
GPIOConfiguration.configure()
refreshInterval = 100      # Update all at this rate
	
pub = rospy.Publisher('gpio_msgs/values',GPIOState,queue_size=1)
rospy.init_node('sb_publish_gpio_values')
rate = rospy.Rate(1)  # 1hz response time
# Specify all the args whether we use them or not.
msg = GPIOState('pins')

# Initialize the pin object fields, including values of outputs
pinlist = GPIOConfiguration.initialize(msg)
for pin in pinlist:
	if pin.mode=="OUT":
		pin.value = -1

count = 0
while not rospy.is_shutdown():
	all = False
	if count%refreshInterval == 0:
		all = True
		count = 0
	count = count + 1
	pins = []
	for pin in pinlist:
		if pin.mode=="OUT":
			val = GPIO.input(pin.channel)
			pin.value=val
			if all or val != pin.value:
				pins.append(pin)

	if len(pins)>0:
		state = GPIOState()
		state.pins = pins
		pub.publish(state)
		rospy.loginfo("Published %d GPIO pin values"%(len(pins)))

	rate.sleep()

rospy.loginfo("complete")
