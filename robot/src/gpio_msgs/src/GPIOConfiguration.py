#!/usr/bin/env python
#
# Configure the GPIO ports as IN or OUT appropriately.
# This should be called on startup.
#
import rospy
import RPi.GPIO as GPIO
from gpio_msgs.msg import GPIOState, GPIOPin

# Get warning if pins are set as outputs, but they are not.
def configure():
	GPIO.setmode(GPIO.BOARD)
	GPIO.setwarnings(False)
	chanlist = [4,11,27]
	for channel in chanlist:
		try:
			GPIO.setup([channel],GPIO.OUT)
			rospy.loginfo("Setting GPIO ",channel," to OUT")
		except:
			rospy.logwarn("ERROR: Setting GPIO ",channel," to OUT")


# Argument is a GPIOState.
def initialize(state):
	count = state.PIN_COUNT
	state.pins = definePins()
	rospy.loginfo("Initialized state with %d pins"%(len(state.pins)))

def definePins():
	pins = []
	pin1 = GPIOPin()
	pin1.label = "3.3V"
	pin1.channel = 1
	pin1.mode = "PWR"
	pins.append(pin1)

	pin2 = GPIOPin()
	pin2.label = "5"
	pin2.channel = 2
	pin2.mode = "PWR"
	pins.append(pin2)

	pin3 = GPIOPin()
	pin3.label = "BCM 2"
	pin3.channel = 3
	pin3.mode = str(GPIO.gpio_function(3))
	pins.append(pin3)

	pin4 = GPIOPin()
	pin4.label = "5V"
	pin4.channel = 4
	pin4.mode = "PWR"
	pins.append(pin4)

	pin5 = GPIOPin()
	pin5.label = "BCM 3"
	pin5.channel = 5
	pin5.mode = str(GPIO.gpio_function(5))
	pins.append(pin5)

	pin6 = GPIOPin()
	pin6.label = "GND"
	pin6.channel = 6
	pin6.mode = "GND"
	pins.append(pin6)

	pin7 = GPIOPin()
	pin7.label = "BCM 4"
	pin7.channel = 7
	pin7.mode = str(GPIO.gpio_function(7))
	pins.append(pin7)

	pin8 = GPIOPin()
	pin8.label = "BCM 14"
	pin8.channel = 8
	pin8.mode = str(GPIO.gpio_function(8))
	pins.append(pin8)

	pin9 = GPIOPin()
	pin9.label = "GND"
	pin9.channel = 9
	pin9.mode = "GND"
	pins.append(pin9)

	pin10 = GPIOPin()
	pin10.label = "BCM 15"
	pin10.channel = 10
	pin10.mode = str(GPIO.gpio_function(10))
	pins.append(pin10)

	pin11 = GPIOPin()
	pin11.label = "BCM 17"
	pin11.channel = 11
	pin11.mode = str(GPIO.gpio_function(11))
	pins.append(pin11)

	pin12 = GPIOPin()
	pin12.label = "BCM 18"
	pin12.channel = 12
	pin12.mode = str(GPIO.gpio_function(12))
	pins.append(pin12)

	pin13 = GPIOPin()
	pin13.label = "BCM 27"
	pin13.mode = "PWR"
	pin13.mode = str(GPIO.gpio_function(13))
	pins.append(pin13)

	pin14 = GPIOPin()
	pin14.label = "GND"
	pin14.channel = 14
	pin14.mode = "GND"
	pins.append(pin14)

	pin15 = GPIOPin()
	pin15.label = "BCM 22"
	pin15.channel = 15
	pin15.mode = str(GPIO.gpio_function(15))
	pins.append(pin15)

	pin16 = GPIOPin()
	pin16.label = "BCM 23"
	pin16.channel = 16
	pin16.mode = str(GPIO.gpio_function(16))
	pins.append(pin16)

	pin17 = GPIOPin()
	pin17.label = "3.3V"
	pin17.channel = 17
	pin17.mode = "PWR"
	pins.append(pin17)

	pin18 = GPIOPin()
	pin18.label = "BCM 24"
	pin18.channel = 18
	pin18.mode = str(GPIO.gpio_function(18))
	pins.append(pin18)

	pin19 = GPIOPin()
	pin19.label = "BCM 10"
	pin19.channel = 19
	pin19.mode = str(GPIO.gpio_function(19))
	pins.append(pin19)

	pin20 = GPIOPin()
	pin20.label = "GND"
	pin20.channel = 20
	pin20.mode = "GND"
	pins.append(pin20)

	pin21 = GPIOPin()
	pin21.label = "BCM 9"
	pin21.channel = 21
	pin21.mode = str(GPIO.gpio_function(21))
	pins.append(pin21)

	pin22 = GPIOPin()
	pin22.label = "BCM 25"
	pin22.channel = 22
	pin22.mode = str(GPIO.gpio_function(22))
	pins.append(pin22)

	pin23 = GPIOPin()
	pin23.label = "BCM 11"
	pin23.channel = 23
	pin23.mode = str(GPIO.gpio_function(23))
	pins.append(pin23)

	pin24 = GPIOPin()
	pin24.label = "BCM 8"
	pin24.channel = 24
	pin24.mode = str(GPIO.gpio_function(24))
	pins.append(pin24)

	pin25 = GPIOPin()
	pin25.label = "GND"
	pin25.channel = 25
	pin25.mode = "GND"
	pins.append(pin25)

	pin26 = GPIOPin()
	pin26.label = "BCM 7"
	pin26.channel = 26
	pin26.mode = str(GPIO.gpio_function(26))
	pins.append(pin26)

	# Got error when tried to set this
	pin27 = GPIOPin()
	pin27.label = "BCM 0"
	pin27.channel = 27
	#pin27.mode = str(GPIO.gpio_function(27))
	pins.append(pin27)

	# Got error when tried to set this
	pin28 = GPIOPin()
	pin28.label = "BCM 1"
	pin28.channel = 28
	#pin28.mode = str(GPIO.gpio_function(28))
	pins.append(pin28)

	pin29 = GPIOPin()
	pin29.label = "BCM 5"
	pin29.channel = 29
	pin29.mode = str(GPIO.gpio_function(29))
	pins.append(pin29)

	pin30 = GPIOPin()
	pin30.label = "GND"
	pin30.channel = 30
	pin30.mode = "GND"
	pins.append(pin30)

	pin31 = GPIOPin()
	pin31.label = "BCM 6"
	pin31.channel = 31
	pin31.mode = str(GPIO.gpio_function(31))
	pins.append(pin31)

	pin32 = GPIOPin()
	pin32.label = "BCM 12"
	pin32.channel = 32
	pin32.mode = str(GPIO.gpio_function(32))
	pins.append(pin32)

	pin33 = GPIOPin()
	pin33.label = "BCM 13"
	pin33.channel = 33
	pin33.mode = str(GPIO.gpio_function(33))
	pins.append(pin33)

	pin34 = GPIOPin()
	pin34.label = "GND"
	pin34.channel = 34
	pin34.mode = "GND"
	pins.append(pin34)

	pin35 = GPIOPin()
	pin35.label = "BCM 19"
	pin35.channel = 35
	pin35.mode = str(GPIO.gpio_function(35))
	pins.append(pin35)

	pin36 = GPIOPin()
	pin36.label = "BCM 16"
	pin36.channel = 36
	pin36.mode = str(GPIO.gpio_function(36))
	pins.append(pin36)

	pin37 = GPIOPin()
	pin37.label = "BCM 26"
	pin37.channel = 37
	pin37.mode = str(GPIO.gpio_function(37))
	pins.append(pin37)

	pin38 = GPIOPin()
	pin38.label = "BCM 20"
	pin38.channel = 38
	pin38.mode = str(GPIO.gpio_function(38))
	pins.append(pin38)

	pin39 = GPIOPin()
	pin39.label = "GND"
	pin39.channel = 39
	pin39.mode = "GND"
	pins.append(pin39)

	pin40= GPIOPin()
	pin40.label = "BCM `21"
	pin40.channel = 40
	pin40.mode = str(GPIO.gpio_function(40))
	pins.append(pin40)
	
	return pins
