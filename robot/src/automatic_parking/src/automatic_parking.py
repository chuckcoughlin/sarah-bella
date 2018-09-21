#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified clc so that frame of reference is always straight ahead.
# I was unable to find a good target with a high reflective intensity,
# so the parking target is between two towers, e.g. stacked cans.
#################################################################################

# Authors: Gilbert #

import rospy
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from teleop_service.msg import Behavior,TeleopStatus
import numpy as np
import math
import time

MAX_ANGLE   = 0.5
MAX_SPEED   = 0.2
ROBOT_WIDTH = 0.2     # Robot width ~ m
TOLERANCE   = 0.04
TOWER_WIDTH = 0.09    # Approx tower width ~ m
START_OFFSET= 1.0     # Dist from left tower to start ~ m
IGNORE      = 0.02    # Ignore any distances less than this
INFINITY    = 10.


# Position consists of a name, distance and angle from the current
# heading and position of the LIDAR. Distances in meters, angle in radians
class Position(object):
	def __init__(self, name):
		self.name = name
		self.dist = 0.0
		self.angle = 0.0
		self.width = 0   # ~ m

class Parker:
	def __init__(self):
		self.stopped = True

		# Create a Twist message, and fill in the fields.
		self.twist = Twist()
		self.reset = Empty()

		self.step = 0
		# Publish status so that controller can keep track of state
		self.spub = rospy.Publisher('sb_teleop_status',TeleopStatus,queue_size=1)
		self.reset_pub = rospy.Publisher('/reset',Empty,queue_size=1)
		self.msg = TeleopStatus()
		self.report("Parker: initialized.")
		self.behavior = ""
		self.leftTower = Position('left_tower')
		self.rightTower= Position('right_tower')
		self.towerSeparation = -1
		self.targetX = -1.
		self.targetY = -1.


	def report(self,text):
		self.msg.status = text
		self.spub.publish(self.msg)
    
	# Start by finding the parking spot
	def start(self):
		self.stopped = False
		self.report("Parker: started ...");
		# Publish movement commands to the turtlebot's base
		self.pub = rospy.Publisher('/cmd_vel', Twist,queue_size=1)
		self.step = 0
		self.park()

	def stop(self):
		if not self.stopped:
			self.stopped = True
			self.report("Parker: stopped.")


	# =============================== Parking Sequence ========================
	# Note that negative is forward.
	# Raw angles are 0-2*PI. 0 is straight ahead.
	def park(self):
		self.report("Started parking sequence ...")

		while not rospy.is_shutdown() and not self.stopped:
			scan = rospy.wait_for_message("/scan", LaserScan)
			self.find_parking_markers(scan)
			if self.step == 0:
				# We stay in this state until we find the towers...
				self.report("Park0: searching for markers")
				if self.rightTower.valid and self.leftTower.valid:
					rospy.loginfo("=================================")
					rospy.loginfo("|        |   dist    |   angle   |")
					rospy.loginfo('| left  | {0:.2f}| {1:.0f}|'.format(self.leftTower.dist,\
											np.rad2deg(self.leftTower.angle)))
					rospy.loginfo('| right | {0:.2f}| {1:.0f}|'.format(self.rightTower.dist,\
											np.rad2deg(self.rightTower.angle)))
					rospy.loginfo("=================================")
					self.step = 1
				else:
					self.spub.publish("ERROR: Failed to find parking spot")
			elif self.step == 1:
				self.report("Park1: proceed to reference point")
				self.setTarget(0,START_OFFSET+ROBOT_WIDTH)
				if self.twist.linear.x<IGNORE:
					time.sleep(1)
					self.reset_pub.publish(self,reset)
					time.sleep(3)	
					self.pivot(0.0)
					self.stop()
					self.step = 2
			elif self.step == 2:
				self.report("Park2: downwind leg")
				self.setTarget(START_OFFSET,START_OFFSET+ROBOT_WIDTH)
				if self.twist.linear.x<IGNORE:
					self.step = 3
			elif self.step == 3:
				self.report("Park3: base leg")
				self.setTarget(START_OFFSET,1.5*ROBOT_WIDTH)
				if self.twist.linear.x<IGNORE:
					self.step = 4
			elif self.step == 4:
				self.report("Park4: final approach")
				self.setTarget(1.5*ROBOT_WIDTH,1.5*ROBOT_WIDTH)
				if self.twist.linear.x<IGNORE:
					self.step = 5
			elif self.step == 5:
				self.report("Park5: reverse angle")
				self.setTarget(self.towerSeparation/2.,0.)
				self.reverse()
				if self.twist.linear.x<IGNORE:
					self.step = 6
			elif self.step==6:
				self.report("Auto_parking complete.")
				time.sleep(1)
				self.reset_pub.publish(self,reset)
				time.sleep(3)	
				self.pivot(0.0)
			self.pub.publish(self.twist)

	# =============================== End of Steps ========================

	def find_parking_markers(self,scan):
		count = 0
		position = Position('start')
		position.valid = False
		position.dist = INFINITY
		position.angle = 0.
		while not position.valid and count<5:
			position = self.find_next_position(scan,position)
			count = count+1
		pos1 = position
		
		position = Position('start')
		position.valid = False
		position.dist = pos1.dist
		position.angle    = pos1.angle
		while not position.valid and count<5:
			position = self.find_next_position(scan,position)
			count = count+1
		pos2 = position
		if pos1.angle>pos2.angle:
			self.rightTower = pos1
			self.leftTower  = pos2
		else:
			self.leftTower  = pos1
			self.rightTower = pos2
		self.leftTower.name = 'leftTower'
		self.rightTower.name= 'rightTower'
		# If we've never computed distance between, do it and save it
		# Use law of cosines
		if self.towerSeparation<0:
			a = pos1.dist
			b = pos2.dist
			angle = pos1.angle-pos2.angle
			self.towerSeparation = math.fabs(math.sqrt(a*a+b*b-2.*a*b*math.cos(angle)))
			rospy.loginfo("Park: Tower separation {:.2f}".format(self.towerSeparation))
			rospy.loginfo(" a,b,angle: {:.2f} {:.2f} {:.2f}".format(a,b,angle))

	# Search the scan results for the next-closest pillar
	def find_next_position(self,scan,startPosition):
		position = Position('tower')
		delta  = scan.angle_increment
		angle  = scan.angle_min-delta
		for d in scan.ranges:
			angle = angle + delta
			if d>IGNORE and angle>startPosition.angle and d<startPosition.dist:
				position.dist = d
		# Now get the angle span
		minAngle = 0
		maxAngle    = 2*math.pi
		angle  = scan.angle_min-delta
		inArc = False
		a = INFINITY
		b = INFINITY
		for d in scan.ranges:
			angle = angle + delta
			if d>=position.dist and d<position.dist+TOLERANCE:
				minAngle = angle
				inArc = True
			elif inArc:
				maxAngle = angle - delta
				break

		position.angle = (maxAngle - minAngle)/2.
		if position.angle>math.pi:
			position.angle = position.angle-2.*math.pi
		# Use law of cosines to get width of post
		width = math.sqrt(a*a+b*b-2.*a*b*math.cos(maxAngle-minAngle))
		if width<2.*TOWER_WIDTH:
			position.valid = True
			position.width = width
		return position

		

	# Angle is with respect to x-axis (leftTower>rightTower)
	def pivot(self,angle):
		self.twist.linear.x  = 0.0
		self.twist.angular.z = 0.0

	# Compute the current position as a projection on the x-axis
	# We have the LIDAR positions to the tower.
	def projectionX(self):
		# By law of sines, with c as angle at rightTower
		sinc = self.leftTower.dist*math.sin(self.leftTower.angle-self.rightTower.angle)/\
				    					self.towerSeparation
		rospy.loginfo("ProjectionX: {:.2f} {:.2f} {:.2f} {:.2f}".format(\
			sinc,self.leftTower.dist,sin(self.leftTower.angle-self.rightTower.angle),\
			self.towerSeparation)
		cosc = math.sqrt(1. - sinc*sinc)
		return cosc*self.leftTower.dist - self.towerSeparation

	def projectionY(self):
		# By law of sines, with c as angle at rightTower
		sinc = self.leftTower.dist*math.sin(self.leftTower.angle-self.rightTower.angle)/\
				    					self.towerSeparation
		return sinc*self.leftTower.dist

	# Turn toward target. Target 0->2PI.
	def rampedAngle(self):
		angle = self.targetDirection
		if angle>math.pi:
			angle = angle - 2*math.pi
		if angle<0.0:
			if angle<-MAX_ANGLE:
				angle = -MAX_ANGLE
		else:
			if angle>MAX_ANGLE:
				angle = MAX_ANGLE
		return -angle

	def reverse(self):
		self.twist.linear.x = -self.twist.linear.x

	# Specify the target destination in terms of a reference system
	# origin at leftTower with x-axis through rightTower.
	# we start the maneuvers. We travel to this point and pivot.
	# If the left tower is the origin and the right tower on the x-axis,
	# then the reference point is at (0,ROBOT_WIDTH+START_OFFSET)
	def setTarget(self,x,y):
		dx = self.projectionX()
		dy = self.projectionY()
		x = x + dx
		y = y + dy
		self.twist.angular.z = math.atan2(x,y)
		self.twist.linear.x  = math.sqrt(x*x+y*y)
		if self.twist.linear.x<MAX_SPEED:
			self.twist.linear.x = MAX_SPEED


# The overall behavior has changed. Start the folower if state is "follow".
def getBehavior(behavior):
	global parker
	global behaviorName
	behaviorName = behavior.state
	if behavior.state=="park":
		parker.start()
	else:
		parker.stop()

if __name__ == "__main__":
	# Initialize the node
	rospy.init_node('sb_park', log_level=rospy.INFO, anonymous=True)
	parker = Parker()
	behaviorName = ""
	rospy.Subscriber("/sb_behavior",Behavior,getBehavior)

	rospy.spin()

