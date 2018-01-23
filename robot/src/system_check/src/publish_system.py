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
msg= System('','')

while not rospy.is_shutdown():
	msg.hostname = socket.gethostname()
	msg.ip_address = socket.gethostbyname(topic.hostname)
	msg.cpu_percent = psutil.cpu_percent(interval=1)
	mem = psutil.virtual_memory()
	msg.memory_percent_used = mem["percent"]
	msg.free_memory_bytes   = mem["free"]
	msg.swap_memory_percent_used = psutil.swap_memory().get("percent")
	msg.disk_percent_used = psutil.disk_usage("/").get("percent")
	net = psutil.net_io_counters(nowrap=True)
	msg.packets_sent = net["packets_sent"]
	msg.packets_received = net["packets_received"]
	msg.in_packets_dropped = net["dropin"]
	msg.out_packets_dropped = net["dropout"]
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
