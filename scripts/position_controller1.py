#!/usr/bin/env python2

# importing required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import *
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
import math

# global flag variables
flag =[0,0,0]
i1=0
i2=0
i3=0

# regions ={
#         "front" : 0 ,
#         "right" : 1 ,
#         "back" : 2,
#         "left" : 3
#         }

#Creating Class For implementing given task
class edrone():
    # constructor
    #default object constructor-> gets called everytime when an instance is initialzed
    def __init__(self):
        #initiasing ros node with name position_controller
        rospy.init_node('position_controller')

        # initialising set points as passed from parameters
        self.goal_point = [19.0007046575,71.9998955286, 22.1599967919]
        #initialising current points
        self.curr_point = [19.0009248718,71.9998318945, 22.16]
        self.set_point = [19.0009248718,71.9998318945, 22.16]
        self.set_point[2]+=3.00

        #initialsing Roll, Pitch and Yaw 
        self.attitude_cmd = edrone_cmd()
        self.attitude_cmd.rcRoll = 1500.0
        self.attitude_cmd.rcPitch = 1500.0
        self.attitude_cmd.rcYaw = 1500.0
        self.attitude_cmd.rcThrottle = 1500.0
        self.base_value = 1500


        # setting PID constants based on tuning
        # self.Kp = [5000*100,
        # self.Kd = [2000*1000,1486*1000, 1550.0*0.3]
        # self.Ki = [0, 0, 0*0.008]
        self.Kp = [0.06*1000*176, 1243* 0.06*1000, 1082*0.06]
        self.Ki = [0.0, 0.0, 0.0*0.008]
        self.Kd = [0.3*10000*873, 2102*0.3*10000, 4476*0.3]

        # calculating errors
        self.error = [999.00, 999.0, 999.00]
        self.prev_error = [0.0, 0.0, 0.0]
        self.iterm = [0.0, 0.0, 0.0]
        self.pErr = [0.0, 0.0, 0.0]
        self.output = [0.0, 0.0, 0.0]
        self.dErr = [0.0, 0.0, 0.0]
        self.obs_dist = [0.0,0.0,0.0,0.0]
        self.next_point=[0,0,0,0]
        self.qrcode = [0.0,0.0,0.0]
        self.ranges = [0.0,0.0,0.0,0.0]
        self.sample_time = 0.06

        # state 1)takeoff 2)fly 3)land 4)gripper check,pick and drop,QR
        self.drone_state=1
        self.isLoaded=False
        self.fly_hieght=3.0000
        self.gripper=False
        self.check = False
        self.dist = 0.0
        self.t = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.prev_setpoint = [0.0,0.0]

        #--------------------------------------------------------------------------------------------------------------
        # Publishing on Topics - /drone_command , /latitude_error, /longitude_error, /altitude_error
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # The below can be uncommented while tuning latitude, longitude and altitude->
        self.latitude_error_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        self.longitude_error_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)

        # Subscriptions on topics /edrone/gps, /pid_tuning_pitch,. /pid_tuning_roll, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid) 
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid) 
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/edrone/gripper_check',String,self.gripper_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber('/qrscan', qr_code, self.qr_callback)
        # rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

        self.gripper_srv=rospy.ServiceProxy('/edrone/activate_gripper',Gripper)

    #------------------------------------------------------------------------------------------------------------------
    # Callback functions
    def qr_callback(self,msg):
        #print('in qr callback')
        self.qrcode[0]= msg.latitude
        self.qrcode[1]= msg.longitude
        self.qrcode[2]= msg.altitude
        rospy.loginfo('QR: %f %f %f',self.qrcode[0],self.qrcode[1],self.qrcode[2])

    def range_finder_top_callback(self, msg):
        self.ranges[0] = msg.ranges[0]
        self.ranges[1] = msg.ranges[1]
        self.ranges[2] = msg.ranges[2]
        self.ranges[3] = msg.ranges[3]
        # rospy.loginfo("%f %f %f %f ", self.obs_dist[regions["right"]], self.obs_dist[regions["back"]], self.obs_dist[regions["left"]], self.obs_dist[regions["front"]])
    def gripper_callback(self,msg):
        self.gripper=msg.data
    def gps_callback(self, msg):
        self.curr_point[0] = msg.latitude
        self.curr_point[1] = msg.longitude
        self.curr_point[2] = msg.altitude
      
    # the below can be uncommented while tuning latitude, longitude and altitude 
    # def altitude_range_callback(self, msg):
        # self.altitude_range_curr = msg.ranges[0]

    def latitude_set_pid(self, msg):
        self.Kp[0] = msg.Kp * (100)
        self.Ki[0] = msg.Ki * 10
        self.Kd[0] = msg.Kd * 1000

    def longitude_set_pid(self, msg):
        self.Kp[1] = msg.Kp * (100)
        self.Ki[1] = msg.Ki * 10
        self.Kd[1] = msg.Kd * 1000

    def altitude_set_pid(self, msg):
        self.Kp[2] = msg.Kp * 0.2
        self.Ki[2] = msg.Ki * 0.008
        self.Kd[2] = msg.Kd * 0.3
    #     self.altitude_range_Kp = msg.Kp * 0.06
    #     self.altitude_range_Ki = msg.Ki * 0.008
    #     self.altitude_range_Kd = msg.Kd * 0.3

    #--------------------------------------------------------------------------------------------------------------------------
    # main function for executing PID ALGORITHM
    def pid(self):
        self.error[0] = (self.set_point[0] - self.curr_point[0])
        self.error[1] = (self.set_point[1] - self.curr_point[1])
        self.error[2] = self.set_point[2] - self.curr_point[2]

        # calculating proportional error for PID
        self.pErr[0] = self.Kp[0] * (self.error[0])
        self.pErr[1] = self.Kp[1] * (self.error[1])
        self.pErr[2] = self.Kp[2] * (self.error[2])
       
        # calculating cumulative error for PID
        self.iterm[0] = (self.iterm[0] + self.error[0])
        self.iterm[1] = (self.iterm[1] + self.error[1])
        self.iterm[2] = (self.iterm[2] + self.error[2])
 
        #limiting iterm to counter error_sum saturation
        if self.iterm[2]>1:
        	self.iterm[2]=0

        # calculating differential error for PID
        self.dErr[0] = (self.error[0] - self.prev_error[0])
        self.dErr[1] = (self.error[1] - self.prev_error[1])
        self.dErr[2] = (self.error[2] - self.prev_error[2])
    
        # Final PID Equations ->
        self.output[0] = self.pErr[1] + self.Ki[1] * self.iterm[1] + self.Kd[1] * self.dErr[1] 
        self.output[1] = self.pErr[0] + self.Ki[0] * self.iterm[0] + self.Kd[0] * self.dErr[0] 
        self.output[2] = self.pErr[2] + self.Ki[2] * self.iterm[2] + self.Kd[2] * self.dErr[2] 

        # Applying PID to the base_value for roll pitch and yaw
        self.attitude_cmd.rcRoll = self.base_value + self.output[0]
        self.attitude_cmd.rcPitch = self.base_value + self.output[1]
        self.attitude_cmd.rcThrottle = self.base_value + self.output[2]
        self.attitude_cmd.rcYaw = self.base_value
        
        #updating errors
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
        self.prev_setpoint[0] = self.set_point[0]
        self.prev_setpoint[1] = self.set_point[1]    
        #publishing  on /edrone_command topic
        self.drone_cmd_pub.publish(self.attitude_cmd)

        # The below can be uncommented while tuning 
        # self.latitude_error_pub.publish(self.error[0]*1000000)
        # self.longitude_error_pub.publish(self.error[1])
        # self.altitude_error_pub.publish(self.error[2])

    def distance(self, goal_latitude, goal_longitude, initial_latitude, initial_longitude):
        # self.dist = math.sqrt(math.pow((goal_longitude - initial_longitude) , 2) + math.pow((goal_latitude - initial_latitude) , 2))
        # self.t = self.dist/100
        self.dx = (goal_latitude - initial_latitude)/250
        self.dy = (goal_longitude - initial_longitude)/250
        print(self.dx,self.dy)

    def change(self,n):
        ''' state 1=take off, state 2=fly state 3=land state 4=gripper check,pick drop '''
        self.drone_state=n
        rospy.loginfo("changing to drone_state %f ",self.drone_state)
        if self.drone_state ==1:
            self.set_point[0]=self.curr_point[0]
            self.set_point[1]=self.curr_point[1]
            self.set_point[2]=self.curr_point[2]
            if self.goal_point[2]>self.fly_hieght:
                self.set_point[2]+= self.fly_hieght
            else:
                self.set_point[2]=self.fly_hieght
        elif self.drone_state==2:
            #self.set_point[0]=self.goal_point[0]
            #self.set_point[1]=self.goal_point[1]
            self.set_point[2]=self.goal_point[2]
            if self.goal_point[2]>self.curr_point[2]:
                self.set_point[2]+= self.fly_hieght
            else:
                self.set_point[2]=self.curr_point[2]
        elif self.drone_state==3:
            self.set_point[0]=self.goal_point[0]
            self.set_point[1]=self.goal_point[1]
            self.set_point[2]=self.goal_point[2]


    def handler(self):
        if self.drone_state==1:##takeoff
            if abs(self.error[2])<0.01:
                self.distance(self.goal_point[0], self.goal_point[1], self.set_point[0], self.set_point[1])
                self.change(2)
        elif self.drone_state==2:##path plan
            # rospy.loginfo("curr point lat: %f,long: %f altitude: %f",self.curr_point[0],self.curr_point[1],self.curr_point[2])
            if abs(self.goal_point[0]-self.curr_point[0])<0.0000800 and abs(self.goal_point[1]-self.curr_point[1])<0.0000800:
                self.set_point[0]=self.goal_point[0]
                self.set_point[1]=self.goal_point[1]
                if abs(self.goal_point[0]-self.curr_point[0])<0.000000800 and abs(self.goal_point[1]-self.curr_point[1])<0.00000080:
                    self.change(3)
            else:
                self.path_plan()
        elif self.drone_state==3:##land
            rospy.loginfo("curr point lat: %f,long: %f altitude: %f",self.curr_point[0],self.curr_point[1],self.curr_point[2])
            if abs(self.error[2])<0.01:
                self.change(4)   
        elif self.drone_state==4: ##load unload
            if not self.isLoaded:
                self.check_pick_gripper() ##change the goal point in gripper
                self.goal_point= self.qrcode 
                # self.QR()
                print(self.goal_point)
                r.sleep()
                self.change(1)
            else:
                # self.drop_parcel()
                rospy.signal_shutdown("reached")

    def check_pick_gripper(self):
        print(self.gripper)
        while self.gripper=="False":
            print(self.gripper)
            continue
        res=GripperResponse()
        
        while not res.result:
            res = self.gripper_srv(True)
            print("res",res)
        self.isLoaded=True

    def wall_foll_waypoint_gen(self):
        d=10
        print(self.ranges)
        if (self.ranges[0]>d and self.ranges[1]>d and self.ranges[2]>d and 1<self.ranges[3]<=d ):
            self.next_point[0] = self.prev_setpoint[0] 
            self.next_point[1] = self.prev_setpoint[1] + self.dy
            rospy.loginfo("Hello")       
            self.check = True
        elif (1<self.ranges[0]<=d and self.ranges[1]>d and self.ranges[2]>d and self.ranges[3]>d ):
            self.next_point[0] = self.prev_setpoint[0] + self.dx
            self.next_point[1] = self.prev_setpoint[1] 
            rospy.loginfo("Hi")  
        elif (self.ranges[0]>d and 1<self.ranges[1]<=d and self.ranges[2]>d and self.ranges[3]>d ):
            self.next_point[0] = self.prev_setpoint[0] 
            self.next_point[1] = self.prev_setpoint[1] - self.dy
            rospy.loginfo("Bonjour") 
        elif (self.ranges[0]>d and self.ranges[1]>d and 1<self.ranges[2]<=d and self.ranges[3]>d ):
            self.next_point[0] = self.prev_setpoint[0] - self.dx
            self.next_point[1] = self.prev_setpoint[1] 
            rospy.loginfo("Namoste")
        elif (self.ranges[0]>d and self.ranges[1]>d and self.ranges[2]>d and self.ranges[3]>d ):
            if self.check == True :
                self.next_point[0] = self.prev_setpoint[0] 
                self.next_point[1] = self.prev_setpoint[1] 
            else :  
                if (self.goal_point[0]-self.curr_point[0])<0:
                    self.next_point[0] = self.prev_setpoint[0] + self.dx
                else:
                    self.next_point[0] = self.prev_setpoint[0] - self.dx
                if (self.goal_point[1]-self.curr_point[1])>0:
                    self.next_point[1] = self.prev_setpoint[1] + self.dy
                else:
                    self.next_point[1] = self.prev_setpoint[1] - self.dy
                #print ("Doland Trump")             
        #     self.error[0] = 0.0
        #     self.next_point[0] = self.curr_point[0] + (d-regions['front'])
        #     self.next_point[1] = self.curr_point[1] - d
        # elif self.obs_dist[regions['right']]<=d:
        #     self.error[1]= 0.0
        #     self.next_point[0] = self.curr_point[0] - d
        #     self.next_point[1] = self.curr_point[1] - (d-regions['right'])
        # elif self.obs_dist[regions['left']]<=d:
        #     self.error[1]= 0.0
        #     self.next_point[0] = self.curr_point[0] - d 
        #     self.next_point[1] = self.curr_point[1] + (d-regions['left'])
        # else:

    
    def path_plan(self):
        self.wall_foll_waypoint_gen()
        self.set_point[0] = self.next_point[0] 
        self.set_point[1] = self.next_point[1]
        print(self.set_point[0],self.set_point[1])


        
        

# main driver code
if __name__ == '__main__':
    # creating objects for different setpoints
    e_drone = edrone()
    r = rospy.Rate(1 / e_drone.sample_time)  
    # e_drone.change(1)
    #running the PID corresponding to the given setpoints and the limiting error conditions when rospy is sctive->
    while not rospy.is_shutdown():
        e_drone.handler()
        e_drone.pid()
        r.sleep()
        