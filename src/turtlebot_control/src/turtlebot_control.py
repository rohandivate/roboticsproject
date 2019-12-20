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

# def main():
# 	r = rospy.Rate(10)
	

def check_role(color):
	# bot mapping, hard coded
	bot_mapping = {
		'pink': 'ar_marker_4',
		'black': 'ar_marker_3'
	}

	# settings
	stick_frame = 'stick_frame'
	threshold = 0.2

	role_map = {}
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)
	
	count = 0
	active_pursuer = ''
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		try:
			if count < 15:
				xs = []
				for bot_id, bot_tag_frame in bot_mapping.items():
					tf_to_stick = tfBuffer.lookup_transform(bot_tag_frame, 'stick_frame', rospy.Time())
					vec = tf_to_stick.transform.translation
					norm = (vec.x ** 2 + vec.y ** 2 + vec.z ** 2) ** 0.5
					vec = np.array([vec.x, vec.y, vec.z])
					unit_vec = vec/norm

					print('transform', vec)
					print('unit vector', unit_vec)
					print('x', unit_vec[0])

					xs.append((abs(unit_vec[0]), bot_id))

				xs = sorted(xs, key=lambda x: x[0]) # pick the lowest x value
				if xs[0][0] < threshold:
					pursuer, evaders = xs[0], xs[1:]
					if bot_mapping[pursuer[1]] == active_pursuer: # pursuer = (score, color)
						count = count + 1
					else:
						count = 1
						active_pursuer = bot_mapping[pursuer[1]]
				else:
					pursuer, evaders = None, xs
					print('no pursuer')
					count = 0
				print('roles')
				print('pursuer:', pursuer)
				print('evaders: ', evaders)
			else:
				break
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			print(e)
			pass
		r.sleep()
	
	bot_marker = bot_mapping[color]
	print(active_pursuer)
	if bot_marker == active_pursuer:
		return 'pursuer'
	return 'evader'


	
			

		# 	# if 

		# 	# 	# by design, the stick frame x-axis points in the direction of the green sign
		# 	# 	# checking the x-axis value of the normalized transform
		# 	# 	# is equivalent to checking the cosine similarity between
		# 	# 	# the translation vector and the x-axis of the stick frame
		# 	# 	similarity = abs(unit_vec[0])
		# 	# 	if similarity < threshold:
		# 	# 		print('PURSUER')
		# 	# 		role_map[bot_id] = 'pursuer'

		# 	# 	else:
		# 	# 		role_map[bot_id] = 'evader'
		# 	# print()
		# 	# print('ROLE MAP!!!')
		# 	# print(role_map)
		# 	# print()
		# except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
		# 	print(e)
		# 	pass
			
		# r.sleep()
		

#Define the method which contains the main functionality of the node.

def controller(turtlebot_ID, role, pursuer_frame, evader_frame):
	"""
	Inputs:
	- turtlebot_ID: the color of the turtlebot
	- role: current role (0 for evader, 1 for pursuer)
	- pursuer: the tf frame of the AR tag on the turtlebot marked as a pursuer
	- evader frames: array containing tf frames of each of the AR tags on the evader bots
	"""

	print('running controller')

	channel = '/shared/' + turtlebot_ID
	pursuer_pose_channel = '/shared/pursuer_pose/'
	upper_left = 'ar_marker_0'
	lower_right = 'ar_marker_1'
	stick_frame = 'stick_frame'
	game_over_distance = 0.4
	out_of_bounds_threshold = 0.03

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

	# bot_mapping = {
	# 	'pink': 'ar_marker_4',
	# 	'black': 'ar_marker_3'
	# }
	
	# if role == 'pursuer':
	# 	pursuer_frame = bot_mapping[color]
	# 	evader_frame = [v for k,v in bot_mapping.items() if k != color][0]
	# else:
	# 	pursuer_frame = [v for k,v in bot_mapping.items() if k != color][0]
	# 	evader_frame = bot_mapping[color]
	
	print('pursuer frame, outside loop', pursuer_frame)
	print('evader frame, outside loop', evader_frame)
	
	# Loop until the node is killed with Ctrl-C
	# TODO: move this to the wrapper/main script that calls the controller(..) API
	while not rospy.is_shutdown():
		try:
			
			if role == 'pursuer':
				best_trans = None
				min_distance = 100
				# new_evader_frames = []
				# if not evader_frames:
				# 	print('Ending game')
				# 	break # TODO: add reset code

				# for evader_frame in evader_frames: # change this to pick the best evader to chase
				# 	pursuer_trans = tfBuffer.lookup_transform(pursuer_frame, evader_frame, rospy.Time())
				# 	x = pursuer_trans.transform.translation.x
				# 	y = pursuer_trans.transform.translation.y
				# 	dist = (x**2 + y**2) ** 0.5
				# 	print(role, pursuer_frame, evader_frame, dist)

				# 	if dist < min_distance:
				# 		best_trans = pursuer_trans
					# if dist > game_over_distance:
					# 	new_evader_frames.append(evader_frame) # check if evader is now out of the game
				# evader_frames = new_evader_frames # update evader frames 
				best_trans = tfBuffer.lookup_transform(pursuer_frame, evader_frame, rospy.Time())
				x = best_trans.transform.translation.x
				y = best_trans.transform.translation.y
				dist = (x**2 + y**2) ** 0.5
				print('pursuer best trans dist', dist)

				if dist < game_over_distance:
					print('Game over')
					break
				else:	
					control_command = Twist()
					control_command.linear.x = (best_trans.transform.translation.x * K1)
					control_command.angular.z = best_trans.transform.translation.y * K2
					pub.publish(control_command)

				# pursuer publishes its pose to shared channel
				# self_pose = Pose()
				# pursuer_pub.publish(self_pose)
			
			elif role == 'evader':

				# evader_frame = evader_frames[0] # one bot at a time

				#bot_mapping[turtlebot_ID] # ar marker of the current evader

				#set up game space
				# upper_left_boundary = tfBuffer.lookup_transform(evader_frame, upper_left, rospy.Time())
				# lower_right_boundary = tfBuffer.lookup_transform(evader_frame, lower_right, rospy.Time())
				# grid_diagonal = tfBuffer.lookup_transform(lower_right, upper_left, rospy.Time())
				# ul = (abs(upper_left_boundary.transform.translation.x),abs(upper_left_boundary.transform.translation.y))
				# lr = (abs(lower_right_boundary.transform.translation.x),abs(lower_right_boundary.transform.translation.y))

				# diagonal_dist = (grid_diagonal.transform.translation.x**2 + grid_diagonal.transform.translation.y**2)**0.5
				# print('diagonal', diagonal_dist)
				
				# if (ul[0] < out_of_bounds_threshold or ul[1] < out_of_bounds_threshold or lr[0] < out_of_bounds_threshold or lr[1] < out_of_bounds_threshold):
				# 	print(ul)
				# 	print(lr)
				# 	print("out of bounds")
				# 	break
				


				best_trans = None
				min_distance = 100
				best_trans = tfBuffer.lookup_transform(pursuer_frame, evader_frame, rospy.Time())
				print(best_trans)
				
				# x = best_trans.transform.translation.x
				# y = best_trans.transform.translation.y
				# dist = (x**2 + y**2) ** 0.5

				# edge_threshold = 0.1
				# if ul[0] < edge_threshold or ul[1] < edge_threshold or lr[0] < edge_threshold or lr[1] < edge_threshold:
				# 	control_command = Twist()
				# 	control_command.angular.z = 3.14
				# 	control_command.linear.x = 0.02			
				# if dist < game_over_distance:
				# 	print(dist)
				# 	print('Game over')
				# 	break
				# elif dist > diagonal_dist / 2:
				# 	print(dist, 'far')
				# 	control_command = Twist()
				# 	control_command.angular.z = best_trans.transform.translation.y * K2
				# 	pub.publish(control_command)
				# else:
				# print(dist, 'close')	
				
				control_command = Twist()
				x = best_trans.transform.translation.x
				y = best_trans.transform.translation.y
				z = best_trans.transform.translation.z
				dist = (x**2 + y**2 + z**2) ** 0.5
				print(dist)
				if dist < 1:
					control_command.linear.x = best_trans.transform.translation.x * K1
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

	color = sys.argv[1]
	role = check_role(color)
	print('printing role in main', color, role)
	bot_mapping = {
		'pink': 'ar_marker_4',
		'black': 'ar_marker_3'
	}
	
	if role == 'pursuer':
		pursuer_frame = bot_mapping[color]
		evader_frame = [v for k,v in bot_mapping.items() if k != color][0]
	else:
		pursuer_frame = [v for k,v in bot_mapping.items() if k != color][0]
		evader_frame = bot_mapping[color]

	controller(color, role, pursuer_frame, evader_frame)
  except rospy.ROSInterruptException:
  	print('errored')
	pass