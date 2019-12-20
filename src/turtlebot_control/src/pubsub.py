#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import numpy as np
from geometry_msgs.msg import Twist

def get_channel(turtlebot_ID):
	if turtlebot_ID == 'pink':
		channel = '/pink/mobile_base/commands/velocity' # for pink bot
	if turtlebot_ID == 'black': 
		channel = '/black/mobile_base/commands/velocity' # for black turtlebot
	if turtlebot_ID == 'red':
		channel = '/red/mobile_base/commands/velocity' # for red bot
	return channel

#Define the method which contains the main functionality of the node.
def pubsub_velocity(channel):
	"""
	Inputs:
	- turtlebot_ID: the color of the turtlebot
	"""

	# TODO: set shared channel to be a global variable
	# TODO: make a different shared channel for each turtlebot
	shared_channel = '/shared/' + channel # should be the channel from the main script (turtlebot_control.py)
	
	pub = rospy.Publisher(get_channel(channel), Twist, queue_size=10)
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber(shared_channel, Twist, pub.publish)
	rospy.spin()

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  channel = sys.argv[1]
  pubsub_velocity(channel) # rosrun turtlebot_control pubsub.py pink
