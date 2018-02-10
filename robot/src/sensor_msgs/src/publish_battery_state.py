#!/usr/bin/env python
#
# Package: sensor_msgs. Support for publishing various parameters
#                       available directly from the robot.
#                       BatteryStatus is available from the i2c bus.
#
from subprocess import call
import rospy
import struct
import smbus
import sys
from std_msgs.msg import String,Header
from sensor_msgs.msg import BatteryState
# 0 = /dev/i2c-0
# 1 = /dev/i2c-1
# Virtual machine has only device 0
#MAX17043
try:
	bus = smbus.SMBus(1)
except:
	bus = smbus.SMBus(0)

address = 0x36

# Return the voltage in volts
def readVoltage(bus,address=address):
	try:
		read = bus.read_word_data(address,2)
		swapped = struct.unpack("<H",struct.pack(">H",read))[0]
		voltage = swapped * 78.125/1000000
	except:
		voltage = 0.0
	return voltage

# Return the capacity in percent
# Exception is an IOError
def readCapacity(bus,address=address):
	try:
		read = bus.read_word_data(address,4)
		swapped = struct.unpack("<H",struct.pack(">H",read))[0]
		capacity = swapped/256
	except:
		capacity = 0
	return capacity
	
pub = rospy.Publisher('/sensor_msgs',BatteryState,queue_size=1)
rospy.init_node('sb_publish_battery_state')
rate = rospy.Rate(0.10)  # 10 second publish rate
# Specify all the args whether we use them or not.
msg = BatteryState('header','voltage','current','charge','capacity',\
                   'design_capacity','percentage','power_supply_status',\
                   'power_supply_health','power_supply_technology','present',\
                   'cell_voltage','location','serial_number')
# Initialize fields we don't set in the message
msg.header = Header()
msg.current = 0.0
msg.charge = 0
msg.capacity = 0.0
msg.design_capacity = 0.0
msg.power_supply_status = msg.POWER_SUPPLY_STATUS_UNKNOWN
msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_UNKNOWN
msg.power_supply_technology = msg.POWER_SUPPLY_TECHNOLOGY_LIPO
msg.cell_voltage=[]
seq = 0


while not rospy.is_shutdown():
	msg.header.seq = seq
	msg.header.stamp = rospy.Time.now()
	msg.present = True
	msg.voltage = readVoltage(bus)
	msg.percentage = readCapacity(bus)
	pub.publish(msg)
	rate.sleep()
	seq = seq + 1

print "sensor_msgs.publish_battery_state: complete"
