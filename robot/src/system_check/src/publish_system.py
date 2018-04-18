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
from std_msgs.msg import Header
from std_msgs.msg import String

pub = rospy.Publisher('/sb_system',System,queue_size=1)
rospy.init_node('sb_publish_system')
rate= rospy.Rate(0.10) #  10 second publish rate
msg= System('header','hostname', 'ip_address', 'cpu_percent', 'memory_percent_used',\
            'free_memory_bytes', 'swap_memory_percent_used', \
            'disk_percent_used', 'packets_sent', 'packets_received',\
            'in_packets_dropped', 'out_packets_dropped')

while not rospy.is_shutdown():
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = '0'
	msg.header = header
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
	pub.publish(msg)
	# NOTE: publishing sets the header sequence
	#       Log every nth message
	if msg.header.seq % 5 == 0:
		rospy.loginfo("System: cpu: %2f.2%%, packets dropped: %2f.2%%" % (msg.cpu_percent,100.*msg.in_packets_dropped/msg.packets_received))
	rate.sleep()

rospy.loginfo("complete")
