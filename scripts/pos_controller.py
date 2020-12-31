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

detect = "False"
a = 0
i = 0
loc_count = -1
#Creating Class For implementing given task
class edrone():
    # constructor
    #default object constructor-> gets called everytime when an instance is initialzed
    def __init__(self):
        print("init called")
        #initiasing ros node with name position_controller
        rospy.init_node('position_controller')

        # initialising set points as passed from parameters
        self.goal_point = [0,0,0]
        #initialising current points
        self.curr_point = [10,0,0]
        self.set_point = [0,0,0]
        self.initial_point=[18.9992411381,71.9998195495,16.6600228704]


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
        self.qrcode = [0.0,0.0,0.0]

        self.sample_time = 0.06

        self.regions ={
        "right" : 0 ,
        "back" : 1 ,
        "left" : 2,
        "front" : 3
        }

        # state 1)takeoff 2)fly 3)land 4)gripper check,pick and drop,QR
        self.drone_state=1
        self.isLoaded=False
        self.fly_hieght=2.0000
        self.gripper=False
        self.dist = 0.0
        self.t = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.prev_setpoint = [0.0,0.0]
        self.bottom_dist = 0.0
        self.r=rospy.Rate(1/self.sample_time)

        #--------------------------------------------------------------------------------------------------------------
        # Publishing on Topics - /drone_command , /latitude_error, /longitude_error, /altitude_error
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # The below can be uncommented while tuning latitude, longitude and altitude->
        self.latitude_error_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        self.longitude_error_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)
        self.marker_pub = rospy.Publisher('/marker_id', Int8, queue_size=1)

        # Subscriptions on topics /edrone/gps, /pid_tuning_pitch,. /pid_tuning_roll, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid) 
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid) 
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/edrone/gripper_check',String,self.gripper_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bot_callback)
        rospy.Subscriber('/qrscan', qr_code, self.qr_callback)
        # rospy.Subscriber("/edrone/err_x_m", Float64, self.err_xm_callback)
        # rospy.Subscriber("/edrone/err_y_m", Float64, self.err_ym_callback)
        rospy.Subscriber("/detection", Bool, self.detection)
        rospy.Subscriber("/marker_data", MarkerData, self.marker_callback)


        self.gripper_srv=rospy.ServiceProxy('/edrone/activate_gripper',Gripper)

    #------------------------------------------------------------------------------------------------------------------
    # Callback functions
    def qr_callback(self,msg):
        #print('in qr callback')
        self.qrcode[0]= msg.latitude
        self.qrcode[1]= msg.longitude
        self.qrcode[2]= msg.altitude
        rospy.loginfo('QR: %f %f %f',self.qrcode[0],self.qrcode[1],self.qrcode[2])

    def range_finder_bot_callback(self,msg):
        self.bottom_dist = msg.ranges[0]

    def marker_callback(self,msg):
        self.marker_id = msg.marker_id
        self.err_x_m = msg.err_x_m
        self.err_y_m = msg.err_y_m



    def detection(self,msg):
        self.detect = msg.data

    def range_finder_top_callback(self, msg):
        self.obs_dist[self.regions["right"]] = msg.ranges[0]
        self.obs_dist[self.regions["back"]] = msg.ranges[1]
        self.obs_dist[self.regions["left"]] = msg.ranges[2]
        self.obs_dist[self.regions["front"]] = msg.ranges[3]
        # rospy.loginfo("%f %f %f %f ", self.obs_dist[self.regions["right"]], self.obs_dist[self.regions["back"]], self.obs_dist[self.regions["left"]], self.obs_dist[self.regions["front"]])
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
        # print(self.dx,self.dy)

    # def change(self,n,x,y,z):
        # ''' state 1=take off, state 2=fly state 3=land state 4=gripper check,pick drop '''
        # self.drone_state=n
        # rospy.loginfo("changing to drone_state %f ",self.drone_state)
        # if self.drone_state ==1:
        #     self.set_point[0]=self.curr_point[0]
        #     self.set_point[1]=self.curr_point[1]
        #     self.set_point[2]=self.curr_point[2]
        #     if self.goal_point[2]>self.curr_point[2]:
        #         self.set_point[2]= self.fly_hieght+self.goal_point[2]
        #     else:
        #         self.set_point[2]+=self.fly_hieght
        #     # print(self.set_point[2])
        # elif self.drone_state==2:
        #     #self.set_point[0]=self.goal_point[0]
        #     #self.set_point[1]=self.goal_point[1]
        #     self.set_point[2]=self.goal_point[2]
        #     if self.goal_point[2]>self.curr_point[2]:
        #         self.set_point[2]+= self.fly_hieght
        #     else:
        #         self.set_point[2]=self.curr_point[2]
        # elif self.drone_state==3:
        #     self.set_point[0]=self.goal_point[0]
        #     self.set_point[1]=self.goal_point[1]
        #     self.set_point[2]=self.goal_point[2]


    # def handler(self):
        # if self.drone_state==1:##takeoff
        #     if abs(self.error[2])<0.01:
        #         self.distance(self.goal_point[0], self.goal_point[1], self.set_point[0], self.set_point[1])
        #         self.change(2)
        # elif self.drone_state==2:##path plan
        #     # rospy.loginfo("curr point lat: %f,long: %f altitude: %f",self.curr_point[0],self.curr_point[1],self.curr_point[2])
        #     if abs(self.goal_point[0]-self.curr_point[0])<0.0000800 and abs(self.goal_point[1]-self.curr_point[1])<0.0000800:
        #         self.set_point[0]=self.goal_point[0]
        #         self.set_point[1]=self.goal_point[1]
        #         if abs(self.goal_point[0]-self.curr_point[0])<0.000000800 and abs(self.goal_point[1]-self.curr_point[1])<0.00000080:
        #             self.change(3)
        #     else:
        #         self.path_plan()
        # elif self.drone_state==3:##land
        #     # rospy.loginfo("curr point lat: %f,long: %f altitude: %f",self.curr_point[0],self.curr_point[1],self.curr_point[2])
        #     if abs(self.error[2])<0.01:
        #         self.change(4)   
        # elif self.drone_state==4: ##load unload
        #     if not self.isLoaded:
        #         self.check_pick_gripper() ##change the goal point in gripper
        #         # while(self.qrcode[0]==0):
        #         #     rospy.logwarn("waiting for QR")
        #         #     continue
        #         self.goal_point= self.qrcode 
        #         # self.QR()
        #         # print(self.goal_point)
        #         # self.goal_point=[19.00000,72.0000,8.44]
        #         r.sleep()
        #         self.change(1)
        #     else:
        #         # self.drop_parcel()
        #         rospy.signal_shutdown("reached")

    def check_pick_gripper(self):
        print(self.gripper)
        while self.gripper=="False":
            rospy.logwarn("gripping unavailable")
            continue
        res=GripperResponse()
        
        while not res.result:
            res = self.gripper_srv(True)
            rospy.loginfo("Pick up successful")
        self.isLoaded=True

    def wall_foll_waypoint_gen(self):
        if 1<self.obs_dist[self.regions["right"]]<7 and 1<self.obs_dist[self.regions["left"]]<7 and self.obs_dist[self.regions["front"]]>10 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]
        elif 1<self.obs_dist[self.regions["right"]]<7 and 1<self.obs_dist[self.regions["front"]]<10 and self.obs_dist[self.regions["left"]]>7 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]-self.dy/1.25
            rospy.loginfo("R   F")
        elif 1<self.obs_dist[self.regions["left"]]<7 and 1<self.obs_dist[self.regions["front"]]<10 and self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]+self.dy/1.25
            rospy.loginfo("L   F")

        elif 1<self.obs_dist[self.regions["left"]]<7 and self.obs_dist[self.regions["front"]]>10 and self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]+self.dx*1.25
            self.set_point[1]=self.prev_setpoint[1]#+self.dy/1.25
            rospy.loginfo("   L   ")
        elif 1<self.obs_dist[self.regions["right"]]<7 and self.obs_dist[self.regions["front"]]>10 and self.obs_dist[self.regions["left"]]>7 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]+self.dx*1.25
            self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            rospy.loginfo("   R  ")

        elif 1<self.obs_dist[self.regions["front"]]<10 and self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["left"]]>7 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]+self.dy*3
            rospy.loginfo("   F***")


        

    
    def path_plan(self):
        if ((self.obs_dist[0]<7 and self.obs_dist[0]>1)or (self.obs_dist[2]<7 and self.obs_dist[2]>1) or (self.obs_dist[3]<10 and self.obs_dist[3]>1)):
            self.wall_foll_waypoint_gen()
            # rospy.logerr("wall following")
        else:
            self.set_point[0] = self.prev_setpoint[0] + self.dx
            self.set_point[1] = self.prev_setpoint[1] + self.dy
            # if (self.goal_point[0]-self.curr_point[0])<0:
            #     self.set_point[0] = self.prev_setpoint[0] - self.dx
            # else:
            # if (self.goal_point[1]-self.curr_point[1])>0:
            # else:
            #     self.set_point[1] = self.prev_setpoint[1] - self.dy
            # rospy.logwarn("no obstacle")
        self.prev_setpoint[0] = self.set_point[0]
        self.prev_setpoint[1] = self.set_point[1] 
        # print(self.set_point[0],self.set_point[1])


        
        

# main driver code
if __name__ == "__main__":
    # print("init main")
    # creating objects for different setpoints
    lat_setpoint=[18.9993675932,18.9990965928,18.9990965925]
    long_setpoint=[72.0000569892,72.0000664814,71.9999050292 ]
    alt_setpoint=[10.7,10.75,22.2]
    # self.initial_point=[18.9992411381,71.9998195495,16.6600228704]
    e_drone = edrone()
    e_drone.drone_state=0
    r = e_drone.r
    e_drone.prev_setpoint=[18.9992411381,71.9998195495,16.6600228704]
    while not rospy.is_shutdown():
        # e_drone.handler()
        if e_drone.drone_state==0:#change to takeoff -> state=1
            e_drone.set_point[0]=e_drone.initial_point[0]
            e_drone.set_point[1]=e_drone.initial_point[1]
            e_drone.set_point[2]=e_drone.initial_point[2]
            e_drone.set_point[2]+=e_drone.fly_hieght
            e_drone.drone_state=1
        # rospy.loginfo("take off: %f %f %f",e_drone.set_point[0],e_drone.set_point[1],e_drone.set_point[2])
        elif e_drone.drone_state==1:
            if abs(e_drone.error[2])<0.01:
                e_drone.prev_setpoint[0],e_drone.prev_setpoint[1],e_drone.prev_setpoint[2] = e_drone.set_point[0],e_drone.set_point[1],e_drone.set_point[2],
                e_drone.goal_point[0],e_drone.goal_point[1],e_drone.goal_point[2]=lat_setpoint[loc_count+1],long_setpoint[loc_count+1],alt_setpoint[loc_count+1]
                e_drone.drone_state=2
            rospy.loginfo("taking off")
        elif e_drone.drone_state==2:
            rospy.loginfo("take off: %f %f %f",e_drone.set_point[0],e_drone.set_point[1],e_drone.set_point[2])
            e_drone.distance(e_drone.goal_point[0],e_drone.goal_point[1],e_drone.set_point[0],e_drone.set_point[1])
            if loc_count==-1:
                loc_count+=1
                e_drone.set_point[0],e_drone.set_point[1],e_drone.set_point[2]=e_drone.prev_setpoint[0] + e_drone.dx,e_drone.prev_setpoint[1] + e_drone.dy,max(e_drone.set_point[2],alt_setpoint[loc_count]+e_drone.fly_hieght)  
                e_drone.goal_point[0],e_drone.goal_point[1],e_drone.goal_point[2]=lat_setpoint[loc_count],long_setpoint[loc_count],alt_setpoint[loc_count]
            # e_drone.path_plan()
            elif loc_count==0:
                e_drone.set_point[0],e_drone.set_point[1],e_drone.set_point[2]=e_drone.prev_setpoint[0] + e_drone.dx,e_drone.prev_setpoint[1] + e_drone.dy,max(e_drone.set_point[2],alt_setpoint[loc_count]+e_drone.fly_hieght)   

                rospy.logwarn("point 1")
                if abs(e_drone.goal_point[0]-e_drone.curr_point[0])<0.00000500 and abs(e_drone.goal_point[1]-e_drone.curr_point[1])<0.00000500:
                    if a == 0:
                        if 0.99 < e_drone.bottom_dist < 1.01:
                            print("CHAL CHAL AVE")
                            while i < 500 :
                                i = i + 1
                            a = 1
                        else:
                            print("AE PAGAL AURAT")
                            e_drone.set_point[2] = alt_setpoint[loc_count]+1
                            print(e_drone.set_point[2],e_drone.bottom_dist)

                    if a == 1 :
                        e_drone.set_point[2] = e_drone.prev_setpoint[2]+0.1
                        if e_drone.detect == True :
                            a = 2
                            detect = "True"
                    # e_drone.set_point[2]=alt_setpoint[loc_count]+1
                    # e_drone.goal_point[0],e_drone.goal_point[1],e_drone.goal_point[2]=lat_setpoint[loc_count],long_setpoint[loc_count],alt_setpoint[loc_count]+1                   
                    # if e_drone.curr_point == alt_setpoint[loc_count]+1 :
                    #     e_drone.set_point[2]=e_drone.prev_setpoint[2]+8
                    #     e_drone.goal_point[0],e_drone.goal_point[1],e_drone.goal_point[2]=lat_setpoint[loc_count],long_setpoint[loc_count],alt_setpoint[loc_count]+8    
                        #e_drone.set_point[2]=max(alt_setpoint[loc_count]+e_drone.fly_hieght,e_drone.set_point[2])
                    #     e_drone.goal_point[0],e_drone.goal_point[1],e_drone.goal_point[2]=lat_setpoint[loc_count]-(e_drone.err_x_m/110692.0702932625),long_setpoint[loc_count]-(e_drone.err_y_m/105292.0089353767),e_drone.curr_point[2]

                    # elif e_drone.err_x_m < 0 and e_drone.err_y_m < 0 :
                    #     e_drone.goal_point[0],e_drone.goal_point[1],e_drone.goal_point[2]=lat_setpoint[loc_count]-(e_drone.err_x_m/110692.0702932625),long_setpoint[loc_count]-(e_drone.err_y_m/105292.0089353767),e_drone.curr_point[2]                        
                    
                    # elif e_drone.err_x_m > 0 and e_drone.err_y_m < 0 :
                    #     e_drone.goal_point[0],e_drone.goal_point[1],e_drone.goal_point[2]=(lat_setpoint[loc_count]-(e_drone.err_x_m/110692.0702932625)),(long_setpoint[loc_count]-(e_drone.err_y_m/105292.0089353767)),e_drone.curr_point[2]


                    # elif e_drone.err_x_m < 0 and e_drone.err_y_m > 0 :
                    #     e_drone.goal_point[0],e_drone.goal_point[1],e_drone.goal_point[2]=lat_setpoint[loc_count]-(e_drone.err_x_m/110692.0702932625),long_setpoint[loc_count]-(e_drone.err_y_m/105292.0089353767),e_drone.curr_point[2]

                if detect == "True" :
                    print("Aur kya haal")
 
                    e_drone.goal_point[0] = e_drone.curr_point[0]-e_drone.err_x_m/110692.0702932625
                    e_drone.goal_point[1] = e_drone.curr_point[1]-e_drone.err_y_m/105292.0089353767
                    print(e_drone.goal_point[0],e_drone.goal_point[1])
                    try :
                        print("Hello")
                        if -0.2 < e_drone.err_x_m < 0.2 and -0.2 < e_drone.err_y_m < 0.2 :
                            
                            loc_count=1
                            e_drone.marker_pub.publish(loc_count+1)
                            a = 0
                            i = 0
                            #e_drone.set_point[2]=alt_setpoint[loc_count]+9
                            e_drone.goal_point[0],e_drone.goal_point[1],e_drone.goal_point[2]=lat_setpoint[loc_count],long_setpoint[loc_count],alt_setpoint[loc_count]   
                            detect = "False"
                    except Exception as e:
                        print(e)                    

                e_drone.prev_setpoint[0] = e_drone.set_point[0]
                e_drone.prev_setpoint[1] = e_drone.set_point[1]
                e_drone.prev_setpoint[2] = e_drone.set_point[2]

                    # e_drone.set_point[0],e_drone.set_point[1],e_drone.set_point[2]=lat_setpoint[loc_count],long_setpoint[loc_count],max(e_drone.initial_point[2],alt_setpoint[loc_count])+e_drone.fly_hieght

            elif loc_count==1:
                e_drone.set_point[0],e_drone.set_point[1],e_drone.set_point[2]=e_drone.prev_setpoint[0] + e_drone.dx,e_drone.prev_setpoint[1] + e_drone.dy,max(e_drone.set_point[2],alt_setpoint[loc_count]+e_drone.fly_hieght)                

                rospy.logwarn("point 2")
                if abs(e_drone.goal_point[0]-e_drone.curr_point[0])<0.0000050 and abs(e_drone.goal_point[1]-e_drone.curr_point[1])<0.0000050:
                    if a == 0 :
                        if 0.99 < e_drone.bottom_dist < 1.01:
                            print("CHAL CHAL AVE")
                            while i <500 :
                                i+=1
                            a = 1
                        else:
                            print("AE PAGAL AURAT")
                            e_drone.set_point[2] = alt_setpoint[loc_count]+1
                            print(e_drone.set_point[2],e_drone.bottom_dist)

                    elif a == 1 :
                        e_drone.set_point[2] = e_drone.prev_setpoint[2]+0.25
                        if e_drone.detect == True :
                            a = 2
                            detect = "True"

                if detect == "True" :
                    print("Aur kasa kai bhau")

                    e_drone.goal_point[0] = e_drone.curr_point[0]-e_drone.err_x_m/110692.0702932625
                    e_drone.goal_point[1] = e_drone.curr_point[1]-e_drone.err_y_m/105292.0089353767
                    print(e_drone.goal_point[0],e_drone.goal_point[1])
                    if -1 < e_drone.err_x_m < 1 and -1 < e_drone.err_y_m < 1 :
                        e_drone.set_point[2] = alt_setpoint[loc_count]+5
                    try :
                        print("Hello")
                        if -0.2 < e_drone.err_x_m < 0.2 and -0.2 < e_drone.err_y_m < 0.2 :
                            
                            loc_count=2
                            e_drone.marker_pub.publish(loc_count+1)
                            a= 0
                            i = 0
                            #e_drone.set_point[2]=alt_setpoint[loc_count]+9
                            e_drone.goal_point[0],e_drone.goal_point[1],e_drone.goal_point[2]=lat_setpoint[loc_count],long_setpoint[loc_count],alt_setpoint[loc_count]   
                            detect = "False"
                    except Exception as e:
                        print(e)                    
		    #if  e_drone.curr_point[2] == e_drone.goal_point[2] + 1:
                e_drone.prev_setpoint[0] = e_drone.set_point[0]
                e_drone.prev_setpoint[1] = e_drone.set_point[1]
                e_drone.prev_setpoint[2] = e_drone.set_point[2]

            elif loc_count==2:
                rospy.logwarn("point 3")
                e_drone.set_point[0],e_drone.set_point[1],e_drone.set_point[2]=e_drone.prev_setpoint[0] + e_drone.dx,e_drone.prev_setpoint[1] + e_drone.dy,max(e_drone.set_point[2],alt_setpoint[loc_count]+e_drone.fly_hieght)      
                if abs(e_drone.goal_point[0]-e_drone.curr_point[0])<0.0000050 and abs(e_drone.goal_point[1]-e_drone.curr_point[1])<0.0000050:
                    if a == 0 :
                        if 0.99 < e_drone.bottom_dist < 1.01:
                            print("CHAL CHAL AVE")
                            while i <300 :
                                i+=1
                            a = 1
                        else:
                            print("AE PAGAL AURAT")
                            if 0.5 < e_drone.bottom_dist < 5:
                                e_drone.set_point[2] = e_drone.prev_setpoint[2]-0.01
                            else:
                                e_drone.set_point[2] = e_drone.prev_setpoint[2]-0.5
                            print(e_drone.set_point[2],e_drone.bottom_dist)

                    if a == 1 :
                        e_drone.set_point[2] = e_drone.prev_setpoint[2]+0.1
                        if e_drone.detect == True :
                            a = 2
                            detect = "True"


                if detect == "True" :
                    print("Namoste Doland Trump")

                    e_drone.goal_point[0] = e_drone.curr_point[0]-e_drone.err_x_m/110692.0702932625
                    e_drone.goal_point[1] = e_drone.curr_point[1]-e_drone.err_y_m/105292.0089353767
                    print(e_drone.goal_point[0],e_drone.goal_point[1])
                    if -1 < e_drone.err_x_m < 1 and -1 < e_drone.err_y_m < 1 :
                        e_drone.set_point[2] = alt_setpoint[loc_count]+4
                    try :
                        print("Merci")
                        if -0.1 < e_drone.err_x_m < 0.1 and -0.1 < e_drone.err_y_m < 0.1 :
                            
                            e_drone.drone_state=3
                            e_drone.goal_point[2],e_drone.set_point[2]=alt_setpoint[2],alt_setpoint[2]
                            detect = "False"
                    except Exception as e:
                        print(e) 

            else:
                rospy.signal_shutdown("Khatam Tata GoodBye Gaya")

            e_drone.path_plan()
        e_drone.pid()
        r.sleep()
