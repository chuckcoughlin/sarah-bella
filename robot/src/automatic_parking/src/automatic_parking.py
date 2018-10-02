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
PILLAR_WIDTH = 0.09    # Approx tower width ~ m
START_OFFSET= 1.0     # Dist from left tower to start ~ m
IGNORE      = 0.02    # Ignore any distances less than this
INFINITY    = 10.


# A pillar describes the position of a target pillar in coordinates of the LIDAR
# measurements. It carries attributes used in the calculation.
class Pillar:
	def __init__(self):
		self.valid = False
		self.dist = INFINITY
		self.angle= 0.0
		self.width = 0.0
		self.d1 = INFINITY
		self.d2 = INFINITY
		self.a1 = 0.0
		self.a2 = 0.0
		self.inArc = False

	def clone(pillar):
		self.valid = pillar.valid
		self.dist = pillar.dist
		self.angle= pillar.angle
		self.width = pillar.width
		self.d1 = pillar.d1
		self.d2 = pillar.d2
		self.a1 = pillar.a1
		self.a2 = pillar.a2
		self.inArc = pillar.inArc

	# Point represents the start of a group
	def start(dist,angle):
		self.d1 = dist
		self.d2 = dist
		self.a1 = angle
		self.a2 = angle
		self.inArc = True
		self.valid = False

	# Complete ppoint with what we have
	def stop():
		if self.inArc:
			self.dist  = (self.d1+self.d2)/2.
			self.angle = (self.a1+self.a2)/2.
			self.inArc = False
			self.valid = True
			self.width = self.getWidth()
			if self.width<2.*PILLAR_WIDTH or self.width>2*PILLAR_WIDTH:
				rospy.loginfo("Park: Issue: pillar width: {:.2f}".format(width))
				self.valid = False

	# Point represents the continuation or completion of a group
	def append(dist,angle):
		if dist>self.d2+TOLERANCE:
			self.stop()
		else:
			if dist<self.d1:
				self.d1 = dist
			elif dist>d2:
				self.d2 = dist
			self.a2 = angle

	# Compute width of pillar using law of cosines
	# If width is not reasonable, pillar will be discarded.
	def getWidth(self):
		a = self.d1
		b = self.d2
		theta = self.a2 - self.a1
		width = math.sqrt(a*a+b*b-2.*a*b*math.cos(theta))
		return width

# Position consists of a name, distance and angle from the current
# heading and position of the LIDAR. Distances in meters, angle in radians
class Position:
	def __init__(self, name):
		self.valid = False
		self.name = name
		self.dist = 0.0
		self.angle = 0.0

class Parker:
	def __init__(self):
		self.stopped = True

		# Create a Twist message, and fill in the fields.
		self.twist = Twist()
		self.reset = Empty()
		self.msg = TeleopStatus()

		self.step = 0
		# Publish status so that controller can keep track of state
		self.spub = rospy.Publisher('sb_teleop_status',TeleopStatus,queue_size=1)
		self.reset_pub = rospy.Publisher('/reset',Empty,queue_size=1)
		self.report("Parker: initialized.")
		self.behavior = ""
		self.leftTower = Pillar()
		self.rightTower= Pillar()
		self.towerSeparation = -1
		self.targetX = -1.
		self.targetY = -1.


	def report(self,text):
		if self.msg.status!=text:
			self.msg.status = text
			self.spub.publish(self.msg)
    
	# Start by finding the parking spot
	# We start here every time.
	def start(self):
		self.stopped = False
		self.report("Parker: started ...");
		# Publish movement commands to the turtlebot's base
		self.pub = rospy.Publisher('/cmd_vel', Twist,queue_size=1)
		self.step = 0
		self.sub  = rospy.Subscriber("/scan_throttle",LaserScan,self.getScan)

	def stop(self):
		if not self.stopped:
			self.stopped = True
			self.report("Parker: stopped.")
			self.sub.unregister()

	# Receive a "throttled" scan message (once per second)
	def getScan(self,scan):
		global behaviorName
		if rospy.is_shutdown() or self.stopped:
			return
		if not behaviorName=="park":
			self.stop()
		else:
			# Proceed with application
			self.find_parking_markers(scan)
			if self.rightTower.valid and self.leftTower.valid:
				self.park()
			else:
				rospy.loginfo("Park: Failed to find towers")



	# =============================== Parking Sequence ========================
	# Note that negative is forward.
	# Raw angles are 0-2*PI. 0 is straight ahead.
	def park(self):
		if self.step == 0:
			self.report("Park0: searching for markers")
			rospy.loginfo(' towers:  {:.2f} {1:.0f}, {:.2f} {:.0f}'.format(\
				self.leftTower.dist, np.rad2deg(self.leftTower.angle),\
				self.rightTower.dist,np.rad2deg(self.rightTower.angle)))
			self.step = 1
			return  		
		elif self.step == 1:
			self.report("Park1: proceed to reference point")
			self.setTarget(0,START_OFFSET+ROBOT_WIDTH)
			if self.twist.linear.x<IGNORE:
				self.reset_pub.publish(self,reset)
				self.step = 2
				return
		elif self.step == 2:
			self.report("Park2: pivot")
			self.pivot(0.0)
			self.step = 3
		elif self.step == 3:
			self.report("Park3: downwind leg")
			self.setTarget(START_OFFSET,START_OFFSET+ROBOT_WIDTH)
			if self.twist.linear.x<IGNORE:
				self.step = 4
		elif self.step == 4:
			self.report("Park4: base leg")
			self.setTarget(START_OFFSET,1.5*ROBOT_WIDTH)
			if self.twist.linear.x<IGNORE:
				self.step = 5
		elif self.step == 5:
			self.report("Park5: final approach")
			self.setTarget(1.5*ROBOT_WIDTH,1.5*ROBOT_WIDTH)
			if self.twist.linear.x<IGNORE:
				self.step = 6
		elif self.step == 6:
			self.report("Park6: reverse diagonal")
			self.setTarget(self.towerSeparation/2.,0.)
			self.reverse()
			if self.twist.linear.x<IGNORE:
				self.step = 7
		elif self.step==7:
			self.report("Auto_parking complete.")
			self.reset_pub.publish(self,reset)
			self.pivot(0.0)
			self.stop()

		self.pub.publish(self.twist)

	# =============================== End of Steps ========================

	# Search the scan results for the two closest pillars.
	# Reject objects that are too narrow or wide.
	# Handle the wrap by postulating a third pillar. We may combine.
	def find_parking_markers(self,scan):
		delta  = scan.angle_increment
		angle  = scan.angle_min-delta
		pillar1 = Pillar()  # Closest
		pillar2 = Pillar()  # Next closest
		pillar3 = Pillar()  # In case of a wrap around origin.
		for d in scan.ranges:
			angle = angle + delta
			if d < IGNORE:
				continue
			# New minimum, process no matter what
			if d<pillar1.d1-TOLERANCE:
				pillar1.stop()
				pillar3.clone(pillar2)
				pillar2.clone(pillar1)
				pillar1.start(d,angle)
				rospy.loginfo("Park: Pillar1 start arc at "+str(angle))
			elif pillar1.inArc:
				pillar1.append(d,angle) 
				rospy.loginfo("Park: Pillar1 in arc at "+str(angle))
			elif d<pillar2.d1-TOLERANCE:
				# Farther than pillar1, but closer than current pillar2
				pillar2.stop()
				pillar3.clone(pillar2)
				pillar2.start(d,angle)
				rospy.loginfo("Park: Pillar2 start arc at "+str(angle))
			elif pillar2.inArc:
				pillar2.append(d,angle)
				rospy.loginfo("Park: Pillar2 in arc at "+str(angle))
			elif d<pillar3.d1-TOLERANCE
				pillar3.stop()
				pillar3.start(d,angle)
				rospy.loginfo("Park: Pillar3 start arc at "+str(angle))
			elif pillar3.inArc: 
				pillar3.append(d,angle)
				rospy.loginfo("Park: Pillar3 in arc at "+str(angle))

		
		# Check for wrap-around
		if pillar1.inArc:
			if pillar2.a1 == 0:
				pillar1.combine(pillar2)
				pillar2 = pillar3
			else pillar1.stop()
		elif pillar2.inArc:
			if pillar1.a1==0:
				pillar2.combine(pillar1)
				pillar2 = pillar3
			elif pillar3.a1==0
				pillar2.combine(pillar3)

		# Now assign the tower positions
		if pillar1.angle<pillar2.angle:
			self.leftTower  = pillar1
			self.rightTower = pillar2
		else:
			self.leftTower  = pillar2
			self.rightTower = pillar1

		rospy.loginfo("Park: towers  {:.2f} {:.0f}, {:.2f} {:.0f}".format(\
				self.leftTower.dist,180*self.leftTower.angle/math.pi,\
				self.rightTower.dist,180*self.rightTower.angle/math.pi))

		# If we've never computed distance between, do it and save it
		# Use law of cosines again
		if self.rightTower.valid and self.leftTower.valid \
		  and  self.towerSeparation<0:
			a = pillar1.dist
			b = pillar2.dist
			angle = pillar1.angle-pillar2.angle
			self.towerSeparation = math.fabs(math.sqrt(a*a+b*b-2.*a*b*math.cos(angle)))
			rospy.loginfo("Park: Tower separation {:.2f}".format(self.towerSeparation))
			time.sleep(0.1)
			rospy.loginfo(" a,b,angle: {:.2f} {:.2f} {:.2f}".format(a,b,angle))
			if self.towerSeparation<3*ROBOT_WIDTH:
				self.towerSeparation = 3*ROBOT_WIDTH


		

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
			sinc,self.leftTower.dist,math.sin(self.leftTower.angle-self.rightTower.angle),\
			self.towerSeparation))
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

