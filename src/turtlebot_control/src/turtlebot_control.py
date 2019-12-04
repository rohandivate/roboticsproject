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

#Define the method which contains the main functionality of the node.
def controller(turtlebot_ID, role, pursuer_frame, evader_frames):
	"""
	Inputs:
	- turtlebot_ID: the color of the turtlebot
	- role: current role (0 for evader, 1 for pursuer)
	- pursuer: the tf frame of the AR tag on the turtlebot marked as a pursuer
	- evader frames: array containing tf frames of each of the AR tags on the evader bots
	"""

	channel = '/mobile_base/commands/velocity'
	if turtlebot_ID == 'black': 
		channel = '/black/mobile_base/commands/velocity' # for black turtlebot

	"""Create a publisher and a tf buffer, which is primed with a tf listener"""
	pub = rospy.Publisher(channel, Twist, queue_size=10)
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)

	# Create a timer object that will sleep long enough to result in
	# a 10Hz publishing rate
	r = rospy.Rate(10) # 10hz

	K1 = 0.3
	K2 = 1
	print('ks done')


	# Loop until the node is killed with Ctrl-C
	# TODO: move this to the wrapper/main script that calls the controller(..) API
	while not rospy.is_shutdown():
		print('in while')
		try:
			if role == 'pursuer':
				# TODO: Move this to main script, do all the transformation calculation outside the bots
				best_trans = None
				for evader_frame in evader_frames: # change this to pick the best evader to chase
					best_trans = tfBuffer.lookup_transform(pursuer_frame, evader_frame, rospy.Time())
				trans = best_trans
				print(role, best_trans) # TODO: read from shared channel to get the best_trans
				# Process trans to get your state error
				# Generate a control command to send to the robot
				control_command = Twist()# Generate this
				control_command.linear.x = trans.transform.translation.x * K1
				control_command.angular.z = trans.transform.translation.y * K2


				#################################### end your code ###############

				pub.publish(control_command)
			else:
				best_trans = None
				for evader_frame in evader_frames: # change this to pick the best evader to chase
					best_trans = tfBuffer.lookup_transform(pursuer_frame, evader_frame, rospy.Time())
				trans = best_trans

				# Process trans to get your state error
				# Generate a control command to send to the robot
				control_command = Twist()# Generate this
				control_command.linear.x = trans.transform.translation.x * K1
				control_command.angular.z = trans.transform.translation.y * K2


				#################################### end your code ###############

				pub.publish(control_command)

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			print(e)
			pass
		# Use our rate object to sleep until it is time to publish again
		r.sleep()

	  
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)

  try:
	# rosrun turtlebot_control turtlebot_control.py black pursuer ar_marker_8 ar_marker_2 ... 
	controller(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:])
  except rospy.ROSInterruptException:
  	print('errored')
	pass