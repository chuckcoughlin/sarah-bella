#!/usr/bin/env python
#
# Publish the state of the entire GPIO board at once. This is infrequent.
# Package: gpio_msgs. Support for publishing Raspberry Pu GPIO outputs.
#                     On start of the method we configure the GPIO pins.
#
# NOTE: We've tried separate topic messages for GPIOState and GPIOPin,
#       but there appears to be a synchronization issue. Issue is: 
#       "ros could not process inbound connection: topic types do not match".
#       
#		We normally only report changes. However if "/gpio_msgs/publish_all"
#       is set, the entire array is sent (and then parameter reset).
#
from subprocess import call
import rospy
import sys
import RPi.GPIO as GPIO
from std_msgs.msg import String
from gpio_msgs.msg import GPIOState
from gpio_msgs.msg import GPIOPin
import GPIOConfiguration

def str2bool(s):
	return s.lower() in ('true','t','1','yes')

# Configure the GPIO pins.
GPIOConfiguration.configure()

	
pub = rospy.Publisher('/gpio_msgs',GPIOState,queue_size=1)
rospy.init_node('sb_publish_gpio_state')
rate = rospy.Rate(10)  # 10hz response rate
# Specify all the args whether we use them or not.
msg = GPIOState('pins')

# Initialize the pin object fields, including values of outputs
pinlist = GPIOConfiguration.initialize(msg)
for pin in pinlist:
	if pin.mode=="OUT":
		pin.value = GPIO.input(pin.channel)

while not rospy.is_shutdown():
	publish = str2bool(rospy.get_param("/gpio_msgs/publish_all","False"))
	rospy.set_param("/gpio_msgs/publish_all","False")
	pins = []
	for pin in pinlist:
		if pin.mode=="OUT":
			val = GPIO.input(pin.channel)
			pin.value=val
			if publish or val != pin.value:
				pins.append(pin)
		elif publish:
				pins.append(pin)

	if len(pins)>0:
		state = GPIOState()
		state.pins = pins
		pub.publish(state)

	rate.sleep()

print "gpio_msgs.publish_gpio_state: complete"
