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

INFINITY = 100000.
IGNORE   = 0.20 # Ignore distances less than this

class follower:
	def __init__(self):
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
		self.stopDistance = float(rospy.get_param("/follow/stop_distance",0.3))
		self.followDistance = float(rospy.get_param("/follow/follow_distance",1.0))
		self.maxSpeed = float(rospy.get_param("/follow/max_speed",0.2))
		self.minSpeed = float(rospy.get_param("/follow/min_speed",0.03))

		# Distance to the closest object, and its angular direction, respectively
		self.targetDistance = INFINITY
		self.targetDirection = 0
		# Publish status so that controller can keep track of state
		self.spub = rospy.Publisher('sb_teleop_status',TeleopStatus,queue_size=1)
		self.msg = TeleopStatus()
		self.state=""
		self.reportState("Follower: initialized.")
		self.behavior = ""

	# Follow the closest object until the reset parameter becomes false.
	# NOTE: When publishing, change sign.
	def start(self):
		self.stopped = False
		self.reportState("Follower: started ...");
		# Subscribe to the laser data
		self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
		# Publish movement commands to the turtlebot's base
		self.pub = rospy.Publisher('/cmd_vel', Twist,queue_size=1)

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
			# Finds the closest object (the target)
			self.calculateTarget(scan)

			#if there's something within self.followDist from us, start following.
			if self.targetDistance < self.followDistance and \
			   self.targetDistance>self.stopDistance:
				self.follow()
			else:
				self.doNothing() 

			# Add a log message, so that we know what's going on
			rospy.loginfo('Follower: Target:{:.2f},{:.0f}, command:{:.2f},{:.0f}'.format(\
			    self.targetDistance, 360*self.targetDirection/(2*Math.PI),\
				self.command.linear.x, 360*self.command.angular.z/(2*Math.PI)))
			self.pub.publish(self.command)

	# Follow the nearest object.
	# Note that negative is forward.
	# Raw angles are 0-2*PI. 0 is straight ahead.
	def follow(self):
		self.reportState("following")
		self.command.linear.x = -tanh(5 * (self.stopDistance - self.targetDistance)) * self.maxSpeed
		#turn faster the further we're turned from our intended object.
		self.command.angular.z = -self.targetDirection
		
		#if we're going slower than our min_speed, just stop.
		if abs(self.command.linear.x) < self.minSpeed:
 			self.command.linear.x = 0.0  

	# Stop moving.
	def doNothing(self):
		self.command.linear.x = 0.0
		self.command.angular.z = 0.0
		self.reportState("waiting")

    # Determine the direction of the closest object in radians
	def calculateTarget(self, scan):
		# Only consider distances less than our follow distance
		# Throw readings of zero as bogus
		self.targetDistance = INFINITY
		delta = scan.angle_increment
		angle = scan.angle_max+delta
		for dist in scan.ranges:
			angle = angle - delta
			if dist>IGNORE and dist<self.followDistance and dist<self.targetDistance: 
				self.targetDistance = dist
				self.targetDirection = angle


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

