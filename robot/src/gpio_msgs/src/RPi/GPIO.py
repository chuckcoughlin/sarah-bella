# This is a stub class and MUST NOT exist on the Raspberry Pi.
# Its sole purpose is to allow ROS to run in the Linux virtual machine
# for testing.
BOARD = 0
IN = "IN"
OUT="OUT"

def gpio_function(chan):
	if chan in [4,11,27]:
		return OUT
	else:
		return IN

def input(channel):
	return False

def output(channel,value):
	pass

def setmode(mode):
	pass

def setup(chanlist,mode):
	pass
