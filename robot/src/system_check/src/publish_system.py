#!/usr/bin/env python
#
# Package: system_check. Support for publishing system parameters.
#          These are parameters that help monitor the robot's 
#          health and capabilities.
#
from subprocess import call
import rospy
import socket
from system_check.msg import sb_system
from std_msgs.msg import String

pub = rospy.Publisher('/sb_system',String,queue_size=1)
rospy.init_node('sb_publish_system')
rate= rospy.Rate(5) #  5 second publish rate
topic= sb_system('','')

while not rospy.is_shutdown():
	topic.hostname = socket.gethostname()
	topic.ip_address = socket.gethostbyname(topic.hostname)
	# All args and in-order
	pub.publish([topic.hostname,topic.ip_address])
	rate.sleep()

print "system_check.publish_system: complete"
