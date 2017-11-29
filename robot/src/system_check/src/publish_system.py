#!/usr/bin/env python
#
# Package: system_check. Support for publishing system parameters.
#          These are parameters that help monitor the robot's 
#          health and capabilities.
#
from subprocess import call
import rospy
from system_check.msg import sb_system

rospy.init_node('sb_publish_system',sb_system,queue_size = 1)
pub = rospy.Publisher('/sb_system',
rate= rospy.Rate(2) # 2 second publish rate
data= sb_system()

while not rospy.is_shutdown():
	data.hostname = call('hostname')
	data.ip_address = call('hostname -I')
	pub.publist(data)
	rate.sleep()

print "system_check.publish_system: complete"
