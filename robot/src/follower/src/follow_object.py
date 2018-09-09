#!/usr/bin/env python

# Basic code from Tony Pratkanis at: http://ros.org/wiki/turtlebot_follower.
# Ctated for the Pi Robot Project: http://www.pirobot.org
# Copyright (c) 2012 Patyrick Goebel. All rights reserved.
#
# See http://www.gnu.org/licenses/gpl.html
# Use a hyperbolic tangent as a transfer function
# This class is designerd to follow the nearest thing to it.

import rospy
import os
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from teleop_service.msg import Behavior,TeleopStatus
from math import tanh


class follower:
	def __init__(self, followDistance=2, stopDistance=1, max_speed=0.6, min_speed=0.01 ):
		self.stopped = True
		# Create a Twist message, and fill in the fields.  
		self.command = Twist()
		self.command.linear.x = 0.0
		self.command.linear.y = 0.0
		self.command.linear.z = 0.0
		self.command.angular.x = 0.0
		self.command.angular.y = 0.0
		self.command.angular.z = 0.0

		# How close should we get to things, and what are our speeds?
		self.stopDistance = stopDistance
		self.max_speed = max_speed
		self.min_speed = min_speed
		#at what distance do we start following something/someone?
		self.followDist = followDistance

		# Distance to the closest object, and its position in array, respectively.
		self.closest = 0
		self.position = 0
		# Publish status so that controller can keep track of state
		self.spub = rospy.Publisher('sb_teleop_status',TeleopStatus,queue_size=1)
		self.msg = TeleopStatus()
		self.state=""
		self.reportState("Follower: initialized.")
		self.behavior = ""

	# Follow the closest object until the reset parameter becomes false.
	def start(self):
		self.stopped = False
		# Subscribe to the laser data
		self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
		# Publish movement commands to the turtlebot's base
		self.pub = rospy.Publisher('/cmd_vel', Twist,queue_size=1)
		self.reportState("Follower: started ...");

	def stop(self):
		if not self.stopped:
			self.sub.unregister()
			self.stopped = True
			self.reportState("Follower: stopped.")

	def laser_callback(self, scan):
		global behaviorName
		if rospy.is_shutdown() or self.stopped:
			return
		if not behaviorName=="follow":
			self.stop()
		else:
			# Proceed with calculations
			# Finds the closest thing
			self.getPosition(scan)
			rospy.loginfo('position: {0}'.format(self.position))

			#if there's something within self.followDist from us, start following.
			if (self.closest < self.followDist):
				self.follow()
			else:
				self.doNothing() 

			# Add a log message, so that we know what's going on
			rospy.loginfo('Follower: Distance: {0}, speed: {1}, angular: {2}'.format(self.closest, \
				self.command.linear.x, self.command.angular.z))
			self.pub.publish(self.command)

	#Starts following the nearest object.
	def follow(self):
		self.reportState("following")
		self.command.linear.x = tanh(5 * (self.closest - self.stopDistance)) * self.max_speed
		#turn faster the further we're turned from our intended object.
		self.command.angular.z = ((self.position-320.0)/320.0)
		
		#if we're going slower than our min_speed, just stop.
		if abs(self.command.linear.x) < self.min_speed:
 			self.command.linear.x = 0.0  

	# Stop moving.
	def doNothing(self):
		self.command.linear.x = 0.0
		self.command.angular.z = 0.0
		self.reportState("waiting")

	def getPosition(self, scan):
		# Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
		depths = []
		for dist in scan.ranges:
			if dist>0.:    # We get bogus readings of zero
				depths.append(dist)
		#scan.ranges is a tuple, and we want an array.
		fullDepthsArray = scan.ranges[:]

		#If depths is empty that means we're way too close to an object to get a reading.
		#thus establish our distance/position to nearest object as "0".
		if len(depths) == 0:
			self.closest = 0
			self.position = 0
		else:
			self.closest = min(depths)
			self.position = fullDepthsArray.index(self.closest)

	def reportState(self,status):
		if self.state!=status:
			self.state = status
			self.msg.status=status
			self.spub.publish(self.msg)


# The overall behavior has changed. Start the folower if state is "follow".
def getBehavior(behavior):
	global follower
	global behaviorName
	behaviorName = behavior.state
	if behavior.state=="follow":
		follower.start()
	else:
		follower.stop()

if __name__ == "__main__":
	# Initialize the node
	rospy.init_node('sb_follow', log_level=rospy.INFO, anonymous=True)
	follower = follower()
	behaviorName = "follow"
	rospy.Subscriber("/sb_behavior",Behavior,getBehavior)
	
 	rospy.spin()

