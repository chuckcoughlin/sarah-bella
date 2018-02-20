#!/usr/bin/env python
#
# Publish the state of the entire GPIO board at once. This is infrequent.
# Package: gpio_msgs. Support for publishing Raspberry Pu GPIO outputs.
#                     On start of the method we configure the GPIO pins.
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

	
pub = rospy.Publisher('/gpio_msgs',GPIOState,queue_size=1)
rospy.init_node('sb_publish_gpio_state')
rate = rospy.Rate(0.05)  # 20 second publish rate
# Specify all the args whether we use them or not.
msg = GPIOState('pin1','pin2','pin3','pin4','pin5',\
                'pin6','pin7','pin8','pin9','pin10',\
                'pin11','pin12','pin13','pin14','pin15',\
                'pin16','pin17','pin18','pin19','pin20',\
                'pin21','pin22','pin23','pin24','pin25',\
                'pin26','pin27','pin28','pin29','pin30',\
                'pin31','pin32','pin33','pin34','pin35',\
                'pin36','pin37','pin38','pin39','pin40')

# Initialize the pin object fields, including values of outputs
pins = GPIOConfiguration.initialize(msg)
for pin in pins:
	if pin.mode=="OUT":
		pin.value = GPIO.input(pin.channel)

while not rospy.is_shutdown():
	for pin in pins:
		if pin.mode=="OUT":
			pin.value = GPIO.input(pin.channel)

	pub.publish(msg)
	rate.sleep()

print "gpio_msgs.publish_gpio_output: complete"
