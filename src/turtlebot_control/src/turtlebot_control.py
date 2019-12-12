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

from geometry_msgs.msg import Twist, Pose

#Define the method which contains the main functionality of the node.

def controller(turtlebot_ID, role, pursuer_frame, evader_frames):
	"""
	Inputs:
	- turtlebot_ID: the color of the turtlebot
	- role: current role (0 for evader, 1 for pursuer)
	- pursuer: the tf frame of the AR tag on the turtlebot marked as a pursuer
	- evader frames: array containing tf frames of each of the AR tags on the evader bots
	"""

	channel = '/shared/' + turtlebot_ID
	pursuer_pose_channel = '/shared/pursuer_pose/'
	upper_left = 'ar_marker_16'
	lower_right = 'ar_marker_2'
	stick_frame = 'stick_frame'
	game_over_distance = 0.5

	"""Create a publisher and a tf buffer, which is primed with a tf listener"""
	pub = rospy.Publisher(channel, Twist, queue_size=10)
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)
	
	# publish the pursuer's current pose to a shared channel
	pursuer_pub = rospy.Publisher(pursuer_pose_channel, Pose, queue_size=10)

	# Create a timer object that will sleep long enough to result in
	# a 10Hz publishing rate
	r = rospy.Rate(10) # 10hz

	K1 = 0.3
	K2 = 1
	print('ks done')
	
	# Loop until the node is killed with Ctrl-C
	# TODO: move this to the wrapper/main script that calls the controller(..) API
	while not rospy.is_shutdown() and len(evader_frames) > 0:
		try:
			#set up game space
			upper_left_boundary = tfBuffer.lookup_transform(pursuer_frame, upper_left, rospy.Time())
			lower_right_boundary = tfBuffer.lookup_transform(pursuer_frame, lower_right, rospy.Time())
			ul = (abs(upper_left_boundary.transform.translation.x),abs(upper_left_boundary.transform.translation.y))
			lr = (abs(lower_right_boundary.transform.translation.x),abs(lower_right_boundary.transform.translation.y))

			
			# print((trans.transform.translation.x**2)**0.5, ((trans.transform.translation.y)**2)**0.5)
			# print(dist)
			if (ul[0] < 0.1 or ul[1] < 0.1 or lr[0] < 0.1 or lr[1] < 0.1):
				print(ul)
				print(lr)
				print("out of bounds")
			
			if role == 'pursuer':
				best_trans = None
				min_distance = 100
				new_evader_frames = []
				for evader_frame in evader_frames: # change this to pick the best evader to chase
					pursuer_trans = tfBuffer.lookup_transform(pursuer_frame, evader_frame, rospy.Time())
					x = pursuer_trans.transform.translation.x
					y = pursuer_trans.transform.translation.y
					dist = (x**2 + y**2) ** 0.5
					print(role, pursuer_frame, evader_frame, dist)

					if dist < min_distance:
						best_trans = pursuer_trans
					if dist > game_over_distance:
						new_evader_frames.append(evader_frame) # check if evader is now out of the game
				evader_frames = new_evader_frames # update evader frames 
				control_command = Twist()
				
				x = best_trans.transform.translation.x
				y = best_trans.transform.translation.y
				dist = (x**2 + y**2) ** 0.5

				if dist < 0.5:
					print('Game over')

				control_command = Twist()
				control_command.linear.x = (best_trans.transform.translation.x * K1)
				control_command.angular.z = best_trans.transform.translation.y * K2
				pub.publish(control_command)

				# pursuer publishes its pose to shared channel
				self_pose = Pose()
				pursuer_pub.publish(self_pose)
			
			elif role == 'evader':
				best_trans = None
				min_distance = 100
				new_evader_frames = []
				for evader_frame in evader_frames: # change this to pick the best evader to chase
					best_trans = tfBuffer.lookup_transform(pursuer_frame, evader_frame, rospy.Time())
					# pursuer_trans = tfBuffer.lookup_transform(pursuer_frame, evader_frame, rospy.Time())
					# x = pursuer_trans.transform.translation.x
					# y = pursuer_trans.transform.translation.y
					# dist = (x**2 + y**2) ** 0.5
					# print(role, pursuer_frame, evader_frame, dist)

					def read_pursuer_orientation(pose):
						"""Read from shared pursuer pose channel, publish orientation to current evader channel"""
						rotation_command = Twist()
						rotation_command.angular.z = pose.orientation.z
						print('pursuer z', pose.orientation.z)
						self_pose = Pose()
						print('current bot z', turtle_bot_id, self_pose.orientation.z)
						pub.publish(rotation_command)

					rospy.Subscriber(pursuer_pub, Pose, read_pursuer_orientation)
						
					if dist > game_over_distance:
						new_evader_frames.append(evader_frame) # check if evader is now out of the game

				# current strategy: mimic pursuer's motions
				x = best_trans.transform.translation.x
				y = best_trans.transform.translation.y
				dist = (x**2 + y**2) ** 0.5

				if dist < 0.5:
					print('Game over')

				control_command = Twist()
				# self_pose = Pose()
				control_command.linear.x = (best_trans.transform.translation.x * K1)
				control_command.angular.z = best_trans.transform.translation.y * K2
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
	# bot_mapping = {'pink': 'ar_marker_2', 'black': 'ar_marker_8', ...}
	# while not rospy.is_shutdown():
		# try:
		# 	role, pursuer, evaders = check_role(color, bot_mapping)
		# 	^ internal logic: evaders = [i for i in bots if i != pursuer]
		# 	controller(turtle_bot_id, role, pursuer, evaders)
	controller(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:])
  except rospy.ROSInterruptException:
  	print('errored')
	pass