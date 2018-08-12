#!/usr/bin/env python

# basic code from Alex Hubers (no barking)
# We always need these lines in a ROS python node.  They import all of
# the basic ROS functions, and make sure that the Python path is set
# up properly.  If you cut and paste these lines, make sure you change
# the manifest name to point to the one in the package that you're
# writing.  ROS will use whatever manifest you specify, even if it's
# not in the current package.  This can be *really* hard to debug.

"""Code modified by Peter Tran and Rocky Mazorow to include barking function,
as well as intergrating it with move.py and bark.py."""


import roslib; roslib.load_manifest('follower_bark')
import rospy
import numpy as np
import os
from std_msgs.msg import String, Bool

# The laser scan message
from sensor_msgs.msg import LaserScan

# The velocity command message
from geometry_msgs.msg import Twist

# We use a hyperbolic tangent as a transfer function
from math import tanh

# This class will follow the nearest thing to it.
class follower:
    def __init__(self, followDistance=2, stopDistance=1, max_speed=0.6, min_speed=0.01 ):
        # Subscribe to the laser data
        self.sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)

        # Publish movement commands to the turtlebot's base
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)

        # How close should we get to things, and what are our speeds?
        self.stopDistance = stopDistance
        self.max_speed = max_speed
        self.min_speed = min_speed
	#at what distance do we start following something/someone?
	self.followDist = followDistance

	#the distance to the closest object, and its position in array, respectively.
	self.closest = 0
	self.position = 0
        # Create a Twist message, and fill in the fields.  We're also
        # going to fill in the linear.x field, even though we think
        # we're going to set it later, just to be sure we know its
        # value.
        self.command = Twist()
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0

    def laser_callback(self, scan):
	#determines the closest thing to the Robit.	
	self.getPosition(scan)
	rospy.logdebug('position: {0}'.format(self.position))

	#if there's something within self.followDist from us, start following.
	"""Also publishes to bark.py once it begins to follow. This is to get
	the robot to begin barking as it follows the person."""
	if (self.closest < self.followDist):
	    self.pubbark = rospy.Publisher('follow', String)
	    self.pubbark.publish(String("Bark"))
	    self.follow()
	#else just don't run at all.
	else:
	    self.stop() 

        # Add a log message, so that we know what's going on
        rospy.logdebug('Distance: {0}, speed: {1}, angular: {2}'.format(self.closest, self.command.linear.x, self.command.angular.z))
	#Ensure we have only one publish command.
	self.pub.publish(self.command)

	#Starts following the nearest object.
    def follow(self):
	self.command.linear.x = tanh(5 * (self.closest - self.stopDistance)) * self.max_speed
	#turn faster the further we're turned from our intended object.
	self.command.angular.z = ((self.position-320.0)/320.0)
		
	#if we're going slower than our min_speed, just stop.
	if abs(self.command.linear.x) < self.min_speed:
    	    self.command.linear.x = 0.0  

    def stop(self):
        self.command.linear.x = 0.0
	self.command.angular.z = 0.0
	#function to occupy self.closest and self.position

    def getPosition(self, scan):
        # Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
	depths = []
	for dist in scan.ranges:
	    if not np.isnan(dist):
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
	
def listener():
	"""Subscribes to move.py."""
	print "I am listening"
	rospy.Subscriber("move", Bool, callback)
	rospy.spin()

def callback(data):
	"""Checks if the data sent in from move.py is True, which depends on what
	tracking.py sends into move.py. If True, then the robot will begin to follow
	the person tracking.py detects."""
	if str(data) == "data: True":
		follower()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('follow', log_level=rospy.DEBUG, anonymous=True)
    listener()
    #follower = follower()
    #rospy.spin()

