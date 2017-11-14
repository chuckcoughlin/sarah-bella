#!/usr/bin/env python
#
# Package: system_check. Support for publishing system parameters.
#
from subprocess import call
import rospy
from system_check.msg import sb_system_parameters

rospy.init_node('sb_publish_system_parameters',sb_system_parameters,queue_size = 1)
pub = rospy.Publisher('/sb_system_parameters',
rate= rospy.Rate(2) # 2 second publish rate
data= sb_system_parameters()

while not rospy.is_shutdown():
	data.hostname = call('hostname')
	data.ip_address = call('hostname -I')
	pub.publist(data)
	rate.sleep()

print "system_check.publish_system_parameters: complete"
