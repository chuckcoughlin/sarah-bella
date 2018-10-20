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
import threading
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from teleop_service.msg import Behavior,TeleopStatus
import numpy as np
import math
import time

MAX_ANGLE   = 0.4
MAX_LINEAR  = 0.05
VEL_FACTOR  = 1.0     # Multiply distance to get velocity
ROBOT_WIDTH = 0.1     # Robot width ~ m
TOLERANCE   = 0.02    # Variation in consecutive cloud points in group
ANG_TOLERANCE= 0.05   # Directional correction to target considered adequate
POS_TOLERANCE= 0.05   # Distance to target considered close enough
PILLAR_WIDTH = 0.08   # Approx tower width ~ m
LEG_X       = 0.6	  # Length of downwind leg
REFERENCE_Y = 0.5     # Dist from left tower to reference point ~ m
IGNORE      = 0.02    # Ignore any scan distances less than this
INFINITY    = 10.


# A pillar describes the position of a reference pillar in coordinates of the LIDAR
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

	def clone(self,pillar):
		self.valid = pillar.valid
		self.dist = pillar.dist
		self.angle= pillar.angle
		self.width = pillar.width
		self.d1 = pillar.d1
		self.d2 = pillar.d2
		self.a1 = pillar.a1
		self.a2 = pillar.a2

	# Represents the start of a group
	def start(self,dist,angle):
		self.d1 = dist
		self.d2 = dist
		self.a1 = angle
		self.a2 = angle
		self.valid = False

	# Complete the group of points that might be a pillar.
	# Reject if dist greater than one we already have
	def end(self,d):
		self.dist  = (self.d1+self.d2)/2.
		if self.dist>d:
			self.valid = False
		if self.a1 == self.a2:
			self.valid = False
		else:
			self.angle = (self.a1+self.a2)/2.
			self.valid = True
			self.width = self.getWidth()
			if self.width<PILLAR_WIDTH/2.:
				# rospy.loginfo("Park: Issue: pillar width: {:.2f}".format(self.width))
				self.valid = False

	# Represents the continuation or completion of a group
	# Angles are decreasing as we iterate
	def append(self,dist,angle):
		if dist<self.d1:
			self.d1 = dist
		elif dist>self.d2:
			self.d2 = dist
		self.a1 = angle

	# Combine partial pillars separated across zero degrees
	# The argument is the potential pillar at 360 degrees
	def combine(self,pillar):
		if math.fabs(pillar.dist-self.dist) < 2.*TOLERANCE:
			if pillar.d1<self.d1:
				self.d1 = pillar.d1
			if pillar.d2>self.d2:
				self.d2 = pillar.d2
			self.a2 = self.a2+(2.*math.pi-pillar.a2)
			self.end(0)

	# Compute width of pillar using law of cosines
	# If width is not reasonable, "pillar" will be discarded.
	def getWidth(self):
		a = self.d1
		b = self.d2
		theta = self.a2 - self.a1
		width = math.sqrt(a*a+b*b-2.*a*b*math.cos(theta))
		return width

# ==================== Main Class ====================
class Parker:
	def __init__(self):
		self.stopped = True

		# Pose.position and Pose.orintation (a Quaternion)
		# Create a Twist message, and fill in the fields.
		self.pose  = Pose()   # Current position of the robot (raw)
		self.twist = Twist()  # Used to command movement
		self.twist.linear.x = 0.0
		self.twist.linear.y = 0.0
		self.twist.linear.z = 0.0
		self.twist.angular.x = 0.0
		self.twist.angular.y = 0.0
		self.twist.angular.z = 0.0
		self.rate = rospy.Rate(2) # 1/2 sec for now
		self.msg = TeleopStatus()

		# Publish status so that controller can keep track of state
		self.spub = rospy.Publisher('sb_teleop_status',TeleopStatus,queue_size=1)
		self.initialize()
		self.report("Parker: initialized.")
		self.behavior = ""
		self.lock = threading.Lock()

	def initialize(self):
		self.leftPillar = Point() # Raw coordinates
		self.rightPillar= Point() # Raw coordinates
		self.pillarSeparation = 0.0
		# Position/heading refer to current reference coordinates
		# except while move is in process. 
		self.position   = Point()
		self.position.x = 0.0
		self.position.y = 0.0
		self.heading  = 0.0
		self.initialized = False # Pillars not located yet

	def rampedAngle(self,angle):
		if angle>math.pi:
			angle = angle - 2*math.pi
		elif angle<-math.pi:
			angle = angle + 2*math.pi
		angle = MAX_ANGLE  if angle > MAX_ANGLE else angle
		angle = -MAX_ANGLE if angle <-MAX_ANGLE else angle
		return angle

	def report(self,text):
		if self.msg.status!=text:
			self.msg.status = text
			self.spub.publish(self.msg)
    
	def reset(self):
		self.twist.linear.x = 0.0
		self.twist.angular.z = 0.0
		self.pub.publish(self.twist)

	# Start by finding the parking spot
	# We start here every time.
	def start(self):
		self.stopped = False
		self.report("Parker: started ...");
		# Publish movement commands to the turtlebot's base
		self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		self.sub  = rospy.Subscriber("/scan_throttle",LaserScan,self.getScan)
		self.odom = rospy.Subscriber("/odom",Odometry,self.updatePose)
		self.initialize()

	def stop(self):
		self.initialized = False
		if not self.stopped:
			self.stopped = True
			self.report("Parker: stopped.")
			self.sub.unregister()
			self.odom.unregister()

	#
	# Odometry callback. Update the current instantaneous pose
	def updatePose(self,odom):
		global behaviorNameo
		if rospy.is_shutdown() or self.stopped:
			return
		if not behaviorName=="park":
			self.stop()
		else:
			self.pose = odom.pose.pose

	# Receive a "throttled" scan message (once per second)
	# We leave this running just long enough to find the pillars
	# We have had touble with synchronization. Use a lock, ignore 
	# subsequent requests that arrive too quickly.
	def getScan(self,scan):
		global behaviorName
		if rospy.is_shutdown() or self.stopped:
			return
		if not behaviorName=="park":
			self.stop()
		else:
			# Proceed with application
			locked = self.lock.acquire(0)
			try:
				if locked:
					self.report("Park: finding markers")
					if not self.initialized:
						if self.findParkingMarkers(scan):
							self.sub.unregister()
							self.initialized = True
							self.park()
						else:
							self.report("Park: Failed to find towers")
			finally:
				if locked:
					self.lock.release()



	# =============================== Parking Sequence ========================
	# We have discovered and positioned the pillars. Now move through the pattern.
	# Reset the "position" as we finish each leg to the start of the next.
	def park(self):
		self.report("Park: proceeding to reference point")
		self.moveToTarget(0,-REFERENCE_Y-ROBOT_WIDTH,True)
		#self.report("Park: downwind leg")
		#self.moveToTarget(LEG_X,-REFERENCE_Y-ROBOT_WIDTH,True)
		#self.report("Park: base leg")
		#self.moveToTarget(LEG_X,-1.5*ROBOT_WIDTH,True)
		#self.report("Park: final approach")
		#self.moveToTarget(1.5*ROBOT_WIDTH,-1.5*ROBOT_WIDTH,True)
		#self.report("Park: reverse diagonal")
		#self.moveToTarget(self.pillarSeparation/2.,0.,False)
		self.report("Auto_parking complete.")
		self.reset()
		self.stop()
	# =============================== End of Steps ========================

	# Search the scan results for the two closest pillars.
	# Reject objects that are too narrow or wide.
	# Handle the wrap by postulating a third pillar. We may combine.
	# Return True if we've identified the two pillars.
	# scan.angle_min = 0
	# scan.angle.max = 2*pi
	def findParkingMarkers(self,scan):
		delta  = scan.angle_increment
		angle  = 2.*math.pi - scan.angle_min + delta
		pillar1 = Pillar()  	# Closest
		pillar2 = Pillar()  	# Next closest
		potential = Pillar()  	# In case of a wrap around origin of our reference 
		# Raw data is 0>2PI. 0 is straight ahead, counter-clockwise.
		# We subtract from 2*PI for clockwise coordinates.
		# Lidar pulley is toward front of assembly.
		for d in scan.ranges:
			angle = angle - delta
			if d < IGNORE:
				continue
			# We group readings in a potential pillar
			if d>potential.d1-TOLERANCE and d<potential.d2+TOLERANCE:
				potential.append(d,angle)
			else:
				potential.end(pillar2.dist)
				if potential.valid:
					if potential.dist<pillar1.dist:
						if pillar1.valid:
							pillar2.clone(pillar1)
						pillar1.clone(potential)
					elif potential.dist<pillar2.dist:
						pillar2.clone(potential)
				potential.start(d,angle)

		potential.end(pillar2.dist+TOLERANCE)		
		# Check for wrap-around
		if potential.valid:
			if pillar1.a2 >= scan.angle_max-2*delta:
				pillar1.combine(potential)
			elif pillar2.a2 >= scan.angle_max-2*delta:
				pillar2.combine(potential)

		# Now assign the tower positions if two are valid
		if pillar1.valid and pillar2.valid:
			# Choose the pillar to the left as "p1" 
			# angles are 0->2*PI
			# If we subtract and the difference < PI, then lesser is "left"
			delta = pillar1.angle - pillar2.angle
			if math.fabs(delta) < math.pi:
				if pillar1.angle<pillar2.angle:
					self.setReferenceCoordinates(pillar1,pillar2)
				else:
					self.setReferenceCoordinates(pillar2,pillar1)
			else:
				if pillar1.angle>pillar2.angle:
					self.setReferenceCoordinates(pillar1,pillar2)
				else:
					self.setReferenceCoordinates(pillar2,pillar1)
			return True
		else:
			return False


	# First argument is the left pillar. It is the origin of our reference
	# system. Use pillar geometry to set our first leg origin.
	# For calculations, angles A,B,C are at p1,p2 and origin
	# Sides a,b,c are opposite corresponding angles
	def setReferenceCoordinates(self,p1,p2):
		# By law of cosines
		a = p2.dist
		b = p1.dist
		C = math.fabs(p2.angle-p1.angle)
		if C>math.pi:
			C = 2*math.pi - C
		c = math.fabs(math.sqrt(a*a+b*b-2.*a*b*math.cos(C)))
		# Use law of sines
		sinA = math.sin(C)*a/c
		cosA = math.sqrt(1-sinA*sinA)
		sinB = math.sin(C)*b/c
		cosB = math.sqrt(1-sinB*sinB)
		# ====== To the right of the towers ========
		if b*b>a*a+c*c:
			x = b*cosA
			y = -b*sinA
		# ====== To the left of the towers =========
		elif a*a>b*b+c*c:
			x = a*cosB - c
			y = -a*sinB
		# ====== In front of the towers =========
		else:
			x = c - a*cosB
			y = -a*sinB
				
		# Heading between -pi and pi with respect to reference coordinates
		self.heading = -p1.angle - math.atan(x/(-y))
		if self.heading>math.pi:
			self.heading = self.heading - 2*math.pi
		elif self.heading<-math.pi:
			self.heading = self.heading + 2*math.pi
		
		self.rightPillar = Point()
		self.leftPillar  = Point()
		self.pillarSeparation = c
		self.leftPillar.x = 0.0
		self.leftPillar.y = 0.0
		self.rightPillar.x= c
		self.rightPillar.y= 0.0
		self.position.x = x
		self.position.y = y
		#self.report("Park: Initial origin (xy,heading,abc) {:.2f},{:.2f} {:.0f} ({:.2f} {:.0f}, {:.2f} {:.0f}, {:.2f} {:.0f})".format(\
		#		self.position.x, self.position.y,math.degrees(self.heading),\
		#		b,math.degrees(p1.angle),a,math.degrees(p2.angle),c,math.degrees(C)))
		self.rate.sleep()

	# Request the target destination in terms of a reference system
	# with origin at leftTower with x-axis through rightTower.
	# As we start the maneuver, we pivot to the correct direction,
	# then move to the target. All calculations are in terms of 
	# the reference coordinates.
	# Note that pose.position.x is forward, pos.position.y is left.
	def moveToTarget(self,targx,targy,forward):
		# Compute the target direction, distance with respect to the current leg origin
		dx = targx - self.position.x
		dy = targy - self.position.y
		targetHeading = math.atan2(dx,dy) # True target direction from current position
		if not forward:
			targetHeading = targetHeading - math.pi
		yaw = self.quaternionToYaw(self.pose.orientation)
		targetYaw = yaw + targetHeading - self.heading
		self.report("Park: Move {:.0f}->{:.0f} ({:.2f},{:.2f}->{:.2f},{:.2f})".format(\
				math.degrees(self.heading),math.degrees(targetHeading),\
				self.position.x,self.position.y,targx,targy))

		# Avoid the discontinuity at 0
		offset = 0
		if yaw<0 and targetYaw>0:
			offset = -2*math.pi
		elif yaw>0 and targetYaw<0:
			offset= 2*math.pi
		targetYaw = targetYaw+offset
		# First aim the robot at the target coordinates
		# atan2() returns a number between pi and -pi
		dtheta = ANG_TOLERANCE+1
		while math.fabs(dtheta) > ANG_TOLERANCE and not rospy.is_shutdown() and not self.stopped:
			yaw   = self.quaternionToYaw(self.pose.orientation)
			dtheta = self.rampedAngle(yaw - targetYaw)
			rospy.loginfo("Park: rotate {:.0f}->{:.0f} ({:.0f})".format(math.degrees(yaw),\
						math.degrees(targetYaw),math.degrees(dtheta)))
			self.twist.angular.z = dtheta
			self.twist.linear.x  = 0.0
			self.pub.publish(self.twist)
			self.rate.sleep()
		
		# Next move in a straight line to target. We check to see how far we've travelled.
		dist = math.sqrt(dx*dx+dy*dy)
		x0 = self.pose.position.x
		y0 = self.pose.position.y
		remaining = dist
		while math.fabs(remaining)>POS_TOLERANCE and not rospy.is_shutdown() and not self.stopped:
			x1 = self.pose.position.x
			y1 = self.pose.position.y
			remaining = dist - self.euclideanDistance(x0,y0,x1,y1)
			rospy.loginfo("Park: odom at {:.2f},{:.2f} ({:.2f})".format(x1,y1,remaining))

			velocity = dist*VEL_FACTOR
			if velocity>MAX_LINEAR:
				velocity = MAX_LINEAR
			elif velocity<-MAX_LINEAR:
				velocity = -MAX_LINEAR

			if not forward:
				velocity = -velocity

			# Make progress toward destination
			self.twist.angular.z = 0.0  # We assume we've rotated correctly
			self.twist.linear.x  = -velocity
			self.pub.publish(self.twist)
			self.rate.sleep()

			
		# Once we've reached our destination, stop
		self.twist.angular.z = 0.0
		self.twist.linear.x  = 0.0
		self.pub.publish(self.twist)
		self.rate.sleep()

		# Finally, update heading and current position
		self.position.x = targx
		self.position.y = targy
		self.heading = targetHeading

	def euclideanDistance(self,x0,y0,x1,y1):
		a = x1 - x0
		b = y1 - y0
		return math.sqrt(a*a+b*b)

	# From www.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	def quaternionToYaw(self,q):
		t3 = 2.0*(q.w*q.z+q.x*q.y)
		t4 = 1.0 - 2.0*(q.y*q.y+q.z*q.z)
		yaw = -math.atan2(t3,t4)
		return yaw
				


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

