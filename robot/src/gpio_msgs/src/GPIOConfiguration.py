#!/usr/bin/env python
#
# Configure the GPIO ports as IN or OUT appropriately.
# This should be called on startup.
#
import RPi.GPIO as GPIO
from gpio_msgs.msg import GPIOState, GPIOPin

# By default all pins are configured as inputs.
def configure():
	GPIO.setmode(GPIO.BOARD)
	chanlist = [4,11,27]
	GPIO.setup(chanlist,GPIO.OUT)


# Argument is a GPIOState.
def initialize(state):
	pins = []
	state.pin1 = GPIOPin()
	state.pin1.label = "3.3V"
	state.pin1.channel = 1
	state.pin1.mode = "PWR"
	pins.append(state.pin1)

	state.pin2 = GPIOPin()
	state.pin2.label = "5"
	state.pin2.channel = 2
	state.pin2.mode = "PWR"
	pins.append(state.pin2)

	state.pin3 = GPIOPin()
	state.pin3.label = "BCM 2"
	state.pin3.channel = 3
	state.pin3.mode = GPIO.gpio_function(3)
	pins.append(state.pin3)

	state.pin4 = GPIOPin()
	state.pin4.label = "5V"
	state.pin4.channel = 4
	state.pin4.mode = "PWR"
	pins.append(state.pin4)

	state.pin5 = GPIOPin()
	state.pin5.label = "BCM 3"
	state.pin5.channel = 5
	state.pin5.mode = GPIO.gpio_function(5)
	pins.append(state.pin5)

	state.pin6 = GPIOPin()
	state.pin6.label = "GND"
	state.pin6.channel = 6
	state.pin6.mode = "GND"
	pins.append(state.pin6)

	state.pin7 = GPIOPin()
	state.pin7.label = "BCM 4"
	state.pin7.channel = 7
	state.pin7.mode = GPIO.gpio_function(7)
	pins.append(state.pin7)

	state.pin8 = GPIOPin()
	state.pin8.label = "BCM 14"
	state.pin8.channel = 8
	state.pin8.mode = GPIO.gpio_function(8)
	pins.append(state.pin8)

	state.pin9 = GPIOPin()
	state.pin9.label = "GND"
	state.pin9.channel = 9
	state.pin9.mode = "GND"
	pins.append(state.pin9)

	state.pin10 = GPIOPin()
	state.pin10.label = "BCM 15"
	state.pin10.channel = 10
	state.pin10.mode = GPIO.gpio_function(10)
	pins.append(state.pin10)

	state.pin11 = GPIOPin()
	state.pin11.label = "BCM 17"
	state.pin11.channel = 11
	state.pin11.mode = GPIO.gpio_function(11)
	pins.append(state.pin11)

	state.pin12 = GPIOPin()
	state.pin12.label = "BCM 18"
	state.pin12.channel = 12
	state.pin12.mode = GPIO.gpio_function(12)
	pins.append(state.pin12)

	state.pin13 = GPIOPin()
	state.pin13.label = "BCM 27"
	state.pin13.mode = "PWR"
	state.pin13.mode = GPIO.gpio_function(13)
	pins.append(state.pin13)

	state.pin14 = GPIOPin()
	state.pin14.label = "GND"
	state.pin14.channel = 14
	state.pin14.mode = "GND"
	pins.append(state.pin14)

	state.pin15 = GPIOPin()
	state.pin15.label = "BCM 22"
	state.pin15.channel = 15
	state.pin15.mode = GPIO.gpio_function(15)
	pins.append(state.pin15)

	state.pin16 = GPIOPin()
	state.pin16.label = "BCM 23"
	state.pin16.channel = 16
	state.pin16.mode = GPIO.gpio_function(16)
	pins.append(state.pin16)

	state.pin17 = GPIOPin()
	state.pin17.label = "3.3V"
	state.pin17.channel = 17
	state.pin17.mode = "PWR"
	pins.append(state.pin17)

	state.pin18 = GPIOPin()
	state.pin18.label = "BCM 24"
	state.pin18.channel = 18
	state.pin18.mode = GPIO.gpio_function(18)
	pins.append(state.pin18)

	state.pin19 = GPIOPin()
	state.pin19.label = "BCM 10"
	state.pin19.channel = 19
	state.pin19.mode = GPIO.gpio_function(19)
	pins.append(state.pin19)

	state.pin20 = GPIOPin()
	state.pin20.label = "GND"
	state.pin20.channel = 20
	state.pin20.mode = "GND"
	pins.append(state.pin20)

	state.pin21 = GPIOPin()
	state.pin21.label = "BCM 9"
	state.pin21.channel = 21
	state.pin21.mode = GPIO.gpio_function(21)
	pins.append(state.pin21)

	state.pin22 = GPIOPin()
	state.pin22.label = "BCM 25"
	state.pin22.channel = 22
	state.pin22.mode = GPIO.gpio_function(22)
	pins.append(state.pin22)

	state.pin23 = GPIOPin()
	state.pin23.label = "BCM 11"
	state.pin23.channel = 23
	state.pin23.mode = GPIO.gpio_function(23)
	pins.append(state.pin23)

	state.pin24 = GPIOPin()
	state.pin24.label = "BCM 8"
	state.pin24.channel = 24
	state.pin24.mode = GPIO.gpio_function(24)
	pins.append(state.pin24)

	state.pin25 = GPIOPin()
	state.pin25.label = "GND"
	state.pin25.channel = 25
	state.pin25.mode = "GND"
	pins.append(state.pin25)

	state.pin26 = GPIOPin()
	state.pin26.label = "BCM 7"
	state.pin26.channel = 26
	state.pin26.mode = GPIO.gpio_function(26)
	pins.append(state.pin26)

	state.pin27 = GPIOPin()
	state.pin27.label = "BCM 0"
	state.pin27.channel = 27
	state.pin27.mode = GPIO.gpio_function(27)
	pins.append(state.pin27)

	state.pin28 = GPIOPin()
	state.pin28.label = "BCM 1"
	state.pin28.channel = 28
	state.pin28.mode = GPIO.gpio_function(28)
	pins.append(state.pin28)

	state.pin29 = GPIOPin()
	state.pin29.label = "BCM 5"
	state.pin29.channel = 29
	state.pin29.mode = GPIO.gpio_function(29)
	pins.append(state.pin29)

	state.pin30 = GPIOPin()
	state.pin30.label = "GND"
	state.pin30.channel = 30
	state.pin30.mode = "GND"
	pins.append(state.pin30)

	state.pin31 = GPIOPin()
	state.pin31.label = "BCM 6"
	state.pin31.channel = 31
	state.pin31.mode = GPIO.gpio_function(31)
	pins.append(state.pin31)

	state.pin32 = GPIOPin()
	state.pin32.label = "BCM 12"
	state.pin32.channel = 32
	state.pin32.mode = GPIO.gpio_function(32)
	pins.append(state.pin32)

	state.pin33 = GPIOPin()
	state.pin33.label = "BCM 13"
	state.pin33.channel = 33
	state.pin33.mode = GPIO.gpio_function(33)
	pins.append(state.pin33)

	state.pin34 = GPIOPin()
	state.pin34.label = "GND"
	state.pin34.channel = 34
	state.pin34.mode = "GND"
	pins.append(state.pin34)

	state.pin35 = GPIOPin()
	state.pin35.label = "BCM 19"
	state.pin35.channel = 35
	state.pin35.mode = GPIO.gpio_function(35)
	pins.append(state.pin35)

	state.pin36 = GPIOPin()
	state.pin36.label = "BCM 16"
	state.pin36.channel = 36
	state.pin36.mode = GPIO.gpio_function(36)
	pins.append(state.pin36)

	state.pin37 = GPIOPin()
	state.pin37.label = "BCM 26"
	state.pin37.channel = 37
	state.pin37.mode = GPIO.gpio_function(37)
	pins.append(state.pin37)

	state.pin38 = GPIOPin()
	state.pin38.label = "BCM 20"
	state.pin38.channel = 38
	state.pin38.mode = GPIO.gpio_function(38)
	pins.append(state.pin38)

	state.pin39 = GPIOPin()
	state.pin39.label = "GND"
	state.pin39.channel = 39
	state.pin39.mode = "GND"
	pins.append(state.pin39)

	state.pin40= GPIOPin()
	state.pin40.label = "BCM `21"
	state.pin40.channel = 40
	state.pin40.mode = GPIO.gpio_function(40)
	pins.append(state.pin40)

	return pins
