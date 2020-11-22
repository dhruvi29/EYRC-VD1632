#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist , Point
from sensor_msgs.msg import LaserScan
from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import tf
from std_srvs.srv import *

pub = None
message = edrone_cmd() 
message.rcRoll=1500      
message.rcPitch=1500      
message.rcYaw=1500      
message.rcThrottle=1500      
state = 0
yaw=0

prev_error=0
iterm=0
kp=5000*0.2
ki=0*0.008
kd=1550.0*0.3

sample_time=0.06

current_pos = [0,0,0]
desired_pos = [0,0,3.0]


threshold_yaw = math.pi / 90   # error of +-2degrees is allowed
threshold_distance = 0.2       # error of 20cm is allowed

activate=False

drone_orientation_euler=[0,0,0,0]


def go_to_pos_handler(req):
    global activate 
    activate = req.data
# A service sends a message and also expects a response. SetBool has message called data which is of bool type and the response has 2 parts 1) success variable of bool type and message variable which is a string
# when you return it this goes to the client stating that the service is recieved successfuly 
    res = SetBoolResponse()
    res.success = True
    res.message = "done"
    return res

def done():
    global message,pub
    message.rcPitch=1500
    message.rcYaw=1500 
    message.rcThrottle=1000
    pub.publish(message)
    rospy.logout("REACHED!")

def go_straight():
    global current_pos , desired_pos , threshold_yaw , threshold_distance , yaw ,message
    dist_offset = math.sqrt(math.pow((desired_pos[1] - current_pos[1]) , 2) + math.pow((desired_pos[1] - current_pos[1]) , 2))
    desired_yaw = math.atan2( desired_pos[1] - current_pos[1] , desired_pos[0] - current_pos[0] )
    angle_offset = desired_yaw - yaw 
# The below given step is called normalization. This makes the bot orient towards the goal in the least posible rotation
    if angle_offset > math.pi :
        angle_offset = angle_offset - ((2*math.pi*angle_offset)/angle_offset)

    if ( dist_offset > threshold_distance):
        message.rcPitch = 1550
        message.rcYaw = 1500
    else:
        rospy.loginfo("error in yaw is : {}".format(dist_offset))
        change_state(2)
    
    if (math.fabs(angle_offset) > threshold_yaw):
        rospy.loginfo("error in yaw is : {}".format(angle_offset))
        change_state(0)
    


def change_state( n ):
    global state
    state = n
    rospy.loginfo("changed to state [{}] \n".format(n))


def correct_heading():
    global yaw , threshold_yaw , current_pos , desired_pos , message
    desired_yaw = math.atan2( desired_pos[0] - current_pos[0] , desired_pos[0] - current_pos[0] )
    angle_offset = desired_yaw - yaw 
    if angle_offset > math.pi :
        angle_offset = angle_offset - ((2*math.pi*angle_offset)/angle_offset)

    if (math.fabs(angle_offset) > threshold_yaw):
        if angle_offset > 0 :
            message.rcYaw=1550
            message.rcPitch=1500
        else : 
            message.rcYaw=1450
            message.rcPitch=1500

    else :
        rospy.loginfo("error in yaw is : {}".format(angle_offset))
        change_state(1)



def imu_callback(msg):
    global drone_orientation_euler
    drone_orientation_quaternion=[0,0,0,0]
    drone_orientation_quaternion[0] = msg.orientation.x
    drone_orientation_quaternion[1] = msg.orientation.y
    drone_orientation_quaternion[2] = msg.orientation.z
    drone_orientation_quaternion[3] = msg.orientation.w
    (drone_orientation_euler[0], drone_orientation_euler[1], drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([drone_orientation_quaternion[0], drone_orientation_quaternion[1], drone_orientation_quaternion[2], drone_orientation_quaternion[3]])

def pid():
    global desired_pos,current_pos,iterm,prev_error,kp,ki,kd,sample_time
    error = desired_pos[2] - current_pos[2]
    pErr = kp * error
    iterm = iterm + error
    dErr = (error - prev_error) / sample_time
    output = pErr + ki * iterm * sample_time + kd * dErr 
    message.rcThrottle = 1500 + output
    prev_error = error
    pub.publish(message)

def gps_callback(msg):
    global current_pos
    current_pos[0] = 110692.0702932625 * (msg.latitude - 19)
    current_pos[1] = -105292.0089353767 * (msg.longitude - 72)
    current_pos[2] = msg.altitude

    
def main():
    global pub , state
    rospy.init_node("go_to_pos")
    pub=rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
    rospy.Subscriber('/edrone/imu/data', Imu, imu_callback)
    rospy.Subscriber('/edrone/gps', NavSatFix, gps_callback)
    rospy.Service("/serv_go_to_pos", SetBool , go_to_pos_handler)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not activate:
            continue
        else:
            if (state == 0):
                correct_heading()
            elif(state == 1):
                go_straight()
            elif(state == 2):
                done()
            else:
                rospy.logerr("Unknown State")
            pid()
            rate.sleep()

if __name__ == '__main__':
    main()