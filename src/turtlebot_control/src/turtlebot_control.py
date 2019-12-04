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
def controller(turtlebot_ID, pursuer_frame, evader_frames):
  """
  Controls a turtlebot whose position is denoted by pursuer_frame,
  to go to a position denoted by evader_frame
  Inputs:
  - pursuer_frame: the tf frame of the AR tag on your turtlebot
  - evader_frame: the tf frame of the target AR tag
  """

  ################################### YOUR CODE HERE ##############

 

  """pick the right channel (depends on the bot)"""
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
  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      best_trans = None
      for evader_frame in evader_frames: # change this to pick the best evader to chase
        best_trans = tfBuffer.lookup_transform(pursuer_frame, evader_frame, rospy.Time())
      trans = best_trans
      # Process trans to get your state error
      # Generate a control command to send to the robot
      # k_matrix = np.array([[K1,0],[0, K2]])
      # print(k_matrix)
      # translation = np.array([trans.transform.translation.x, trans.transform.translation.y])
      # print(translation)
      # kt=np.dot(k_matrix,translation)




      control_command = Twist()# Generate this
      control_command.linear.x = trans.transform.translation.x * K1
      control_command.angular.z = trans.transform.translation.y * K2


      #################################### end your code ###############

      pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
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
    controller(sys.argv[1], sys.argv[2], sys.argv[3:])
  except rospy.ROSInterruptException:
    pass