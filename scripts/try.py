#!/usr/bin/env python2
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
import rospy
from std_msgs.msg import *
gripper = None
def gripper_callback(msg):
    global gripper
    gripper=msg.data

if __name__ == '__main__':
    rospy.init_node('gripperT')
    gripper_srv=rospy.ServiceProxy('/edrone/activate_gripper',Gripper)
    rospy.Subscriber('/edrone/gripper_check',String,gripper_callback)
    r = rospy.Rate(20)
    r.sleep()  
    if gripper:
        res = gripper_srv(True)
        print("res: ",res)