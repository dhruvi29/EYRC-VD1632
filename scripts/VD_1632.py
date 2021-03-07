#!/usr/bin/env python
#rospy -> client python library for ROS
import rospy
#through Twist Messages we publish velocities to turtlesim node, using /turtle1/cmd_vel topic 
from geometry_msgs.msg import Twist
#node_turtle_revolve subcribes to turtlesim over the topic /turtle1/Pose
from turtlesim.msg import Pose
#math module for using pi
import math

#turtle_stop when true will halt the bot
turtle_stop=False
initializing=True
#initialising pub and sub
pub=1
sub=1
#initial pose object to calculate initial angle
initial=Pose()
#a is the difference between the instantaneous / current orientation and the initial orientation
a=0.0
distance =0
#/turtle1/Pose topic callback
def pose_callback(pose):
	#global access to variables
	global turtle_stop,initializing,initial,flag,a,distance
	# when theta->(-pi,pi)
	if (pose.theta<initial.theta ): a = (pose.theta +2*math.pi)-initial.theta
	else: a = pose.theta-initial.theta
	
	distance = abs(a)    #distance is the absolute value of a
	
	#for the first run
	if initializing:
		#first pose log
		rospy.loginfo(" moving in a circle, distance: %f \n ", distance)
		initial=pose
		initializing=False
		
		#as a->2pi and bot has not stopped this block will run 
	elif (round((2*math.pi-a),2))<=0.01 and not turtle_stop and not initializing:
		#final log values
		rospy.loginfo(" moving in a circle, distance: %f \n ", distance)
		turtle_stop=True
		rospy.loginfo('Reached\n')
		#calling shutdown() to halt the bot -> completion of circular path once
		shutdown()
		
	else:
		#rospy.loginfo(" moving in a circle, distance: %f \n ", distance)
		pass
		
#main driver function
def move_turtle():
	global turtle_stop,pub,sub
	rospy.init_node('node_turtle_revolve', anonymous=True)          #initialising node
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #publishing to /turtle1/cmd_vel
	sub=rospy.Subscriber('/turtle1/pose',Pose, pose_callback)       #subscribing to /turtle1/pose
	vel=Twist()                                                     #velocity topic's message object
	rate = rospy.Rate(100)                                          # rate of 100Hz
	while not rospy.is_shutdown() and not turtle_stop:
	#initialising linear and angular velocites
		vel.linear.x = 0.7
		vel.angular.z = 0.7     #--> v = wr -> r =1
		#publishing the values of velocity to turtlesim_node
		p=pub.publish(vel)
		rate.sleep()
		
def shutdown():
	rospy.signal_shutdown("reached final position")
	
#exception handling
if __name__ == '__main__':
	try:
		move_turtle()                      #calling main driver function
	
	except rospy.ROSInterruptException:
		print('journey ended')             #in case of an exception while running	
