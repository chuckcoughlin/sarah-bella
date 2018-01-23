#!/usr/bin/env python
#
# Package: system_check. Support for publishing system parameters.
#          These are parameters that help monitor the robot's 
#          health and capabilities.
#
from subprocess import call
import psutil
import rospy
import socket
from system_check.msg import System
from std_msgs.msg import String

pub = rospy.Publisher('/sb_system',String,queue_size=1)
rospy.init_node('sb_publish_system')
rate= rospy.Rate(10) #  10 second publish rate
msg= System('hostname', 'ip_address', 'cpu_percent', 'memory_percent_used', 'free_memory_bytes', 'swap_memory_percent_used', \
            'disk_percent_used', 'packets_sent', 'packets_received', 'in_packets_dropped', 'out_packets_dropped')

while not rospy.is_shutdown():
	msg.hostname = socket.gethostname()
	msg.ip_address = socket.gethostbyname(msg.hostname)
	msg.cpu_percent = psutil.cpu_percent(interval=1)
	mem = psutil.virtual_memory()
	msg.memory_percent_used = mem[2]
	msg.free_memory_bytes   = mem[4]
	msg.swap_memory_percent_used = psutil.swap_memory()[3]
	msg.disk_percent_used = psutil.disk_usage("/")[3]
	net = psutil.net_io_counters()
	msg.packets_sent = net[2]
	msg.packets_received = net[3]
	msg.in_packets_dropped = net[6]
	msg.out_packets_dropped = net[7]
	# All args and in-order
	lst = []
	lst.append(msg.hostname)
	lst.append(msg.ip_address)
	lst.append(msg.cpu_percent)
	lst.append(msg.memory_percent_used)
	lst.append(msg.free_memory_bytes)
	lst.append(msg.swap_memory_percent_used)
	lst.append(msg.disk_percent_used)
	lst.append(msg.packets_sent)
	lst.append(msg.packets_received)
	lst.append(msg.in_packets_dropped)
	lst.append(msg.out_packets_dropped)
	pub.publish(lst)
	rate.sleep()

print "system_check.publish_system: complete"
