#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

turtle_stop=False
initializing=True
pub=1
sub=1
initial=Pose()
a=0.0
#/turtle1/Pose topic callback
def pose_callback(pose):
	global turtle_stop,initializing,initial,flag,a
	
	if (pose.theta<initial.theta ): a = (pose.theta +2*math.pi)-initial.theta
        else: a = pose.theta-initial.theta
	
	if initializing:
		rospy.loginfo('initial pos x=%f ,y=%f ,theta=%f\n',pose.x,pose.y,pose.theta)
		initial=pose
		initializing=False
	elif (round((2*math.pi-a),2))<=0.01 and not turtle_stop and not initializing:
		rospy.loginfo('curr pos x=%f ,y=%f ,theta=%f\n',pose.x,pose.y,pose.theta)
		turtle_stop=True
		rospy.loginfo('Reached\n')
		shutdown()
	else:
		rospy.loginfo('curr pos x=%f ,y=%f ,theta=%f\n',pose.x,pose.y,pose.theta)
		pass
		
	
	
def move_turtle():
	global turtle_stop,pub,sub
	rospy.init_node('node_turtle_revolve', anonymous=True)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    	sub=rospy.Subscriber('/turtle1/pose',Pose, pose_callback)
    	vel=Twist()
    	rate = rospy.Rate(100) 
    	while not rospy.is_shutdown() and not turtle_stop:
        	vel.linear.x = 0.7
        	vel.angular.z = 0.7
        	p=pub.publish(vel)
        	rate.sleep()
def shutdown():
	rospy.signal_shutdown("reached final pos")
if __name__ == '__main__':
	try:
		move_turtle()
	except rospy.ROSInterruptException:
	        print('journey ended')
