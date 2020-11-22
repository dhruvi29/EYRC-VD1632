#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from tf import transformations
#Services require their own type of messages known as Service Messages
#SetBool is predefined ServiceMessage which has a variable called data of type bool
from std_srvs.srv import *


region={
    "front" : 0,
    "right" : 0,
    "back"  : 0,
    "left"  : 0,
    "ffront": 0
    }
srv_wall_follow = None
srv_go_to_pos = None
initial_position = Point()
final_position = Point()
current_pos = [0,0,0]
initial_position.x = 110692.0702932625 * (19.000027000- 19)
initial_position.y = -105292.0089353767 * (72.00000 - 72)
final_position.x =  110692.0702932625 * (19.0000000- 19)
final_position.y = -105292.0089353767 * (72.00000 - 72)
state = 0
time_delay = 0
time_sec = 0
activate=False

def activation(req):
    global activate 
    activate = req.data
    # A service sends a message and also expects a response. SetBool has message called data which is of bool type and the response has 2 parts 1) success variable of bool type and message variable which is a string
    # when you return it this goes to the client stating that the service is recieved successfuly 
    res = SetBoolResponse()
    res.success = True
    res.message = "done"
    return res

def dist_from_line(x , y):
    global initial_position , final_position
    m = (final_position.y - initial_position.y)/(final_position.x - initial_position.x)
    numerator = math.fabs(y - initial_position.y - m*x + m*initial_position.x)
    denominator = (math.sqrt(1+math.pow(m , 2)))
    dist = numerator/denominator
    return dist


def change_state(n):
    global state , srv_go_to_pos , srv_wall_follow
    state = n
    rospy.loginfo("Changed to state [{}]".format(n))
    if( n==0 ):
        resp = srv_go_to_pos(True)
        resp = srv_wall_follow(False)
    elif( n==1 ):
        resp = srv_go_to_pos(False)
        resp = srv_wall_follow(True)
    else:
        rospy.loginfo("Invalid State")


def clbk_range_finder(msg):
    global region
    region = {
        "front" : min(msg.ranges[0] , 2 ),
        "right" :  min(msg.ranges[1], 2 ),
        "back" : min(msg.ranges[2], 2 ),
        "left" :   min(msg.ranges[3] , 2 ),
        "ffront" : min(msg.ranges[4] , 2 )
    }

def gps_callback(msg):
    global current_pos
    current_pos[0] = 110692.0702932625 * (msg.latitude - 19)
    current_pos[1] = -105292.0089353767 * (msg.longitude - 72)
    current_pos[2] = msg.altitude

#state 0 = go_to_point
#state 1 = wall_follow
def main():
    global srv_wall_follow , srv_go_to_pos , state , current_pos , region , time_delay , time_sec
    rospy.init_node("wallfollow_plus_gotopos")
    rospy.Subscriber('/edrone/gps', NavSatFix, gps_callback)
    rospy.Subscriber("/edrone/range_finder_top",LaserScan,clbk_range_finder)

    srv_wall_follow = rospy.ServiceProxy("/serv_wall_follow" , SetBool)
    srv_go_to_pos = rospy.ServiceProxy("/serv_go_to_pos", SetBool)

    rospy.Service("/Bug2",SetBool,activation)

    #We assume that initially there are no obstacle
    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not activate:
            continue 
        dist = dist_from_line(current_pos[0] , current_pos[1])
        if (state == 0):
            if (region["front"] < 1.0 and region["front"] > 0.15):
                time_delay=0
                time_sec = 0
                change_state(1)
        if (state == 1):
            if (time_sec > 9 and dist < 0.2 ):
                change_state(0)
        time_delay = time_delay+1
        if time_delay == 20 :
            time_sec = time_sec+1
            time_delay=0
        rate.sleep()


if __name__ == "__main__" :
    main()
