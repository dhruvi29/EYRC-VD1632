#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_srvs.srv import *
from vitarana_drone.msg import *

activate=False

region ={
        "right" : 0 ,
        "center" : 0,
        "left" :  0 
        }

function = {
        0 : "Find wall" , 
        1 : "Turn left",
        2 : "Follow wall"
        }
pub = None
message = edrone_cmd() 
message.rcRoll=1500      
message.rcPitch=1500      
message.rcYaw=1500      
message.rcThrottle=1500      
status=''
state = 0

prev_error=0
iterm=0
kp=5000*0.2
ki=0*0.008
kd=1550.0*0.3

sample_time=0.06

current_pos = [0,0,0]
set_point = [0,0,3.0]

def gps_callback(msg):
    global current_pos
    current_pos[0] = 110692.0702932625 * (msg.latitude - 19)
    current_pos[1] = -105292.0089353767 * (msg.longitude - 72)
    current_pos[2] = msg.altitude

def wall_follow_handler(req):
    global activate 
    activate = req.data
# A service sends a message and also expects a response. SetBool has message called data which is of bool type and the response has 2 parts 1) success variable of bool type and message variable which is a string
# when you return it this goes to the client stating that the service is recieved successfuly 
    res = SetBoolResponse()
    res.success = True
    res.message = "done"
    return res


def change_state(n):
    global state , function
    state = n
    rospy.loginfo("State Changed to {0}  {1}".format(n , function[state]))


def do_task():
    global region , state , status
    d=1.5
#Our aim here is to keep the right side of the bot near to the obstacle

#case1 Find the wall by going ahead and turning right
    if(region["right"] > d and region["center"] > d and region["left"] > d):
        if activate:
            change_state(0)
#case 2 if the obstacle is to the right then continue following the obstacle
    elif(region["right"] < d and region["center"] > d and region["left"] > d):
        change_state(2)
#case 3 if the obstacle is detected by center region of the laserscan then turn left so that case 2 becomes true and bot follows the wall
    elif(region["right"] > d and region["center"] < d and region["left"] > d):
        change_state(1)
#case 4 if the obstacle is to the left then turn the bot left till case 3 becomes True then indirectly case 2 becomes True and the bot will follow the wall with its right side nearest to it
    elif(region["right"] > d and region["center"] > d and region["left"] < d):
        change_state(1)
#case 5 if right and center are detecting wall then turn left so that right side is nearest to wall and then follow the wall
    elif(region["right"] < d and region["center"] < d and region["left"] > d):
        change_state(1)
#case 6 this is intermediate step while performing case 4
    elif(region["right"] > d and region["center"] < d and region["left"] < d):
        change_state(1)
#case 7 if the bot in between 2 obstacles then find the wall i.e start turning right so that case 2 becomes true and you follow the right wall
    elif(region["right"] < d and region["center"] > d and region["left"] < d):
        change_state(0)

def find_wall():
    global message
    message.rcPitch=1550
    message.rcYaw = 1525

def go_left():
    global message
    message.rcPitch = 1500
    message.rcYaw= 1550

def follow_wall():
    global message
    message.rcPitch=1550
    message.rcYaw=1500

def pid():
    global set_point,current_pos,iterm,prev_error,kp,ki,kd,sample_time
    error = set_point[2] - current_pos[2]
    pErr = kp * error
    iterm = iterm + error
    dErr = (error - prev_error) / sample_time
    output = pErr + ki * iterm * sample_time + kd * dErr 
    message.rcThrottle = 1500 + output
    prev_error = error
    pub.publish(message)

def clbk_range_finder(msg):
    global region
    region = {
        "center" : min(msg.ranges[0] , 2 ),
        "right" :  min(msg.ranges[1], 2 ),
        "left" :   min(msg.ranges[3] , 2 ),
    }
    do_task()

def main():
    global pub
    rospy.init_node("wall_follow")

    rospy.Subscriber("/edrone/range_finder_top",LaserScan,clbk_range_finder)
    rospy.Subscriber('/edrone/gps', NavSatFix, gps_callback)

    pub=rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
    rospy.Service("/serv_wall_follow" , SetBool , wall_follow_handler)
    

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not activate:
            continue
        else:
            if ( state == 0 ):
                find_wall()
            elif( state == 1 ):
                go_left()
            elif( state == 2):
                follow_wall()
            pid()
        rate.sleep()


if __name__ == '__main__':
    main()