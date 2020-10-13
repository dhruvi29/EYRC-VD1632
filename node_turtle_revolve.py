#!/usr/bin/env python

#team eYRC#1632
#Team Members: Dhairya Jigar Shah  Ninad Sunil Jangle Karthik Swaminathan Dhruvi Rashmin Doshi 

# rospy is a client python library for ros  
import rospy
# through Twist topic we will publish the velocities to the turtlebot
from geometry_msgs.msg import Twist
#our node will subscribe to the topic Pose to get the location, orientation and speeds of the bot at every (1/rate)th second 
from turtlesim.msg import Pose
#importing math module to use math.pi value for pi calculations.
import math

#turtle_stop when true will halt the bot
turtle_stop=False
#to store the initial pose values.
initializing=True
#initializing pub and sub with random values
pub=1
sub=1
#initializing initial as an obj of Pose() 
initial=Pose()
#a is the difference between the current orientation(theta) and the initial theta
a=0.0

#/turtle1/Pose topic callback
def pose_callback(pose):
	#defining all these variables as global so we can access them in other functions as well
	global turtle_stop,initializing,initial,flag,a
	# as the theta goes from -pi,pi : initialising a according to the sign of theta
	if (pose.theta<initial.theta ): a = (pose.theta +2*math.pi)-initial.theta
        else: a = pose.theta-initial.theta
	
	#if its the first run
	if initializing:
		#first pose log
		rospy.loginfo('initial pos x=%f ,y=%f ,theta=%f\n',pose.x,pose.y,pose.theta)
		initial=pose
		#making initializing False so the program wont run through this if again
		initializing=False
	#this block will halt the program
	# as a ->2pi, atleast second run and the turtle has not already stopped the block will run
	elif (round((2*math.pi-a),2))<=0.01 and not turtle_stop and not initializing:
		#logging final pose values
		rospy.loginfo('curr pos x=%f ,y=%f ,theta=%f\n',pose.x,pose.y,pose.theta)
		turtle_stop=True
		rospy.loginfo('Reached\n')
		#calling shutdown() to halt the bot
		shutdown()
	else:
		rospy.loginfo('curr pos x=%f ,y=%f ,theta=%f\n',pose.x,pose.y,pose.theta)
		pass
		
	
#main operation tank of the program, publishing, subscribing , value assignment 
def move_turtle():
	global turtle_stop,pub,sub
	#initializing a node
	rospy.init_node('node_turtle_revolve', anonymous=True)
	#publishing to topic cmd_vel
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	#subscribing to topic pose
    	sub=rospy.Subscriber('/turtle1/pose',Pose, pose_callback)
    	#vel is a reference to Twist() topic
    	vel=Twist()
    	rate = rospy.Rate(100) #100Hz 
    	#Ctrl + C not pressed and turtle hasnt already stopped.
    	while not rospy.is_shutdown() and not turtle_stop:
    		#initializing the linear and angular speeds of the bot
        	vel.linear.x = 0.7
        	vel.angular.z = 0.7
        	#publishing the values in vel to cmd_vel
        	p=pub.publish(vel)
        	#execution sleeping/ delay 
        	rate.sleep()
#halts the execution of the progam
def shutdown():
	#printing reached msg along with stopping the bot
	rospy.signal_shutdown("reached final pos")

#FIRST BLOCK TO BE EXECUTED
if __name__ == '__main__':
	#implementing exception handling
	try:
		#calling function to initialize nodes, publish, subscribe,..
		move_turtle()
	except rospy.ROSInterruptException:
		#in case an exception occurs
	        print('journey ended')
