#!/usr/bin/env python
#
# Use the LasarScan to compute the distance to any obstacle in a region in front
# of the robot. The region has a width specified as a parameter.
#
# Package: teleop_service. Width parameter = "robot/width"
#          Refresh interval used to slow down the publication rate
#
import rospy
import sys
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from teleop_service.msg import ObstacleDistance
	
DEFAULT_WIDTH = 1000. # mm
REFRESH_INTERVAL = 2
REPORT_INTERVAL  = 20
INFINITY = 100000.
count = 0


# Specify all the args whether we use them or not.
msg = ObstacleDistance('distance')
pub = rospy.Publisher('sb_obstacle_distance',ObstacleDistance,queue_size=1)
rospy.init_node('sb_publish_obstacle_distance')

# Callback for every new LaserScan message
def callback(laser):
	global count
	count = count+1
	if count%REFRESH_INTERVAL == 0:
		width = float(rospy.get_param("/robot/width",DEFAULT_WIDTH))
		distance = INFINITY
		# Only consider -90 to 90. (0 is straight ahead)
		delta = laser.angle_increment
		angle = laser.angle_min - delta
		for d in laser.ranges:
			angle = angle + delta
			if angle>-math.pi/2 and angle<math.pi/2:
				obstacle = width/2*math.cos(angle)
				if obstacle<distance:
					distance = obstacle

		msg.distance = distance
		pub.publish(msg)
		if count%REPORT_INTERVAL == 0:
			rospy.loginfo("Obstacle distance: %4.2f "%(distance))
	

sub = rospy.Subscriber("/scan_throttle",LaserScan,callback)
rospy.spin()
