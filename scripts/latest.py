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
import csv

# global flag variables
i = 0

#Creating Class For implementing given task
class edrone():
    # constructor
    #default object constructor-> gets called everytime when an instance is initialzed
    def __init__(self):
        print("init called")
        #initiasing ros node with name position_controller
        rospy.init_node('position_controller')

        # initialising set points as passed from parameters

        #initialising current points
        self.curr_point = [19.0,72.0, 8.440994388]
        self.set_point = [19.0,72.0, 8.440994388]
        self.initial_point=[19.0,72.0, 8.440994388]
        self.goal_point = [18.9999860999,71.9999433333, 8.44100417359]
        self.B2 = [19.000000001,71.999957578,8.44100417359]
        self.C1 = [19.000013553,71.9999433333,8.44100417359]
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
        self.Kp = [0.06*4000*176, 1243* 0.06*4000, 1082*0.06]
        self.Ki = [0.0, 0.0, 0.0*0.008]
        self.Kd = [0.3*18000*873, 2102*0.3*17000, 4476*0.3]
        # calculating errors
        self.error = [999.00, 999.0, 999.00]
        self.prev_error = [0.0, 0.0, 0.0]
        self.iterm = [0.0, 0.0, 0.0]
        self.pErr = [0.0, 0.0, 0.0]
        self.output = [0.0, 0.0, 0.0]
        self.dErr = [0.0, 0.0, 0.0]
        self.obs_dist = [0.0,0.0,0.0,0.0,0.0]
        self.qrcode = [0.0,0.0,0.0]

        self.sample_time = 0.06
        self.sensor = ""
        self.regions ={
        "front" : 0 ,
        "right" : 1 ,
        "back" : 2,
        "left" : 3,
        "top" : 4
        }

        # state 1)takeoff 2)fly 3)land 4)gripper check,pick and drop,QR
        self.drone_state=0
        self.isLoaded=False
        self.fly_hieght=3.0000
        self.gripper=False
        self.dist = 0.0
        self.t = 0.0
        self.dist = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.prev_setpoint = [19.0,72.0, 8.440994388]
        self.r=rospy.Rate(1/self.sample_time)

        self.detect = False
        self.lat_setpoint=[]
        self.long_setpoint=[]
        self.alt_setpoint=[]
        self.loc_count=0
        self.isDetected=False
        self.stopDetection=False
        self.check=False
        with open('/home/karthikswami/catkin_ws/src/vitarana_drone/scripts/manifest.csv', 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                row.pop(0)
                self.lat_setpoint.append(float(row[0]))
                self.long_setpoint.append(float(row[1]))
                self.alt_setpoint.append(float(row[2]))
            print(self.lat_setpoint,self.long_setpoint,self.alt_setpoint)


        #--------------------------------------------------------------------------------------------------------------
        # Publishing on Topics - /drone_command , /latitude_error, /longitude_error, /altitude_error
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # The below can be uncommented while tuning latitude, longitude and altitude->
        self.latitude_error_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        self.longitude_error_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)
        self.altitude_pub = rospy.Publisher('/altitude', Float32, queue_size=1)

        # Subscriptions on topics /edrone/gps, /pid_tuning_pitch,. /pid_tuning_roll, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid) 
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid) 
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/edrone/gripper_check',String,self.gripper_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber('/qrscan', qr_code, self.qr_callback)
        rospy.Subscriber("/edrone/err_x_m", Float64, self.err_xm_callback)
        rospy.Subscriber("/edrone/err_y_m", Float64, self.err_ym_callback)
        rospy.Subscriber("/isDetected",Bool,self.detection_clbk)
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

    def err_xm_callback(self,msg):
        self.err_x_m = msg.data

    def err_ym_callback(self,msg):
        self.err_y_m = msg.data

    def range_finder_top_callback(self, msg):
        self.obs_dist[self.regions["front"]] = msg.ranges[0]
        self.obs_dist[self.regions["right"]] = msg.ranges[1]
        self.obs_dist[self.regions["back"]] = msg.ranges[2]
        self.obs_dist[self.regions["left"]] = msg.ranges[3]
        self.obs_dist[self.regions["top"]] = msg.ranges[4]
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
    def detection_clbk(self,msg):
        if not self.stopDetection:
            self.isDetected=msg.data
            # print(self.isDetected)
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

    def distance(self):
        self.dist = math.sqrt(math.pow(110692.0702932625 * (self.goal_point[0] - self.set_point[0]) , 2) + math.pow(105292.0089353767 * (self.goal_point[1] - self.set_point[1]) , 2))
        if 0<self.dist < 3 :
            self.t = self.dist*100
        elif 3 <= self.dist < 10 :
            self.t = self.dist*50
        elif 10 <= self.dist < 40 :
            self.t = self.dist*20
        elif self.dist ==0:
            self.t = 5
        else :
            self.t = self.dist*15
        self.dx = (self.goal_point[0] - self.set_point[0])/self.t
        self.dy = (self.goal_point[1] - self.set_point[1])/self.t

    def change(self,n):
        ''' state 1=take off, state 2=fly state 3=land state 4=gripper check,pick drop '''
        self.drone_state=n
        rospy.loginfo("changing to drone_state %f ",self.drone_state)
        if self.drone_state ==1:
            self.set_point[0]=self.curr_point[0]
            self.set_point[1]=self.curr_point[1]
            self.set_point[2]=self.curr_point[2]
            if self.goal_point[2]>self.curr_point[2]:
                self.set_point[2]= self.fly_hieght+self.goal_point[2]
            else:
                self.set_point[2]+=self.fly_hieght
        elif self.drone_state==2:
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
        global i
        if self.drone_state==0:#change to takeoff -> state=1
            self.set_point[2]=self.curr_point[2]
            if self.goal_point[2]>self.curr_point[2]:
                self.set_point[2]= self.fly_hieght+self.goal_point[2]
            else:
                self.set_point[2]+=self.fly_hieght
            self.drone_state=1
        elif self.drone_state==1:
            self.isLoaded = False
            if abs(self.error[2])<0.1:
                self.prev_setpoint[0],self.prev_setpoint[1],self.prev_setpoint[2] = self.set_point[0],self.set_point[1],self.set_point[2]
                if self.gripper == "True":
                    self.set_point[2]=max(self.set_point[2],self.alt_setpoint[self.loc_count]+self.fly_hieght)
                    self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.loc_count],self.long_setpoint[self.loc_count],self.alt_setpoint[self.loc_count]
                self.distance()                
                self.change(2)
        elif self.drone_state==2:
            if self.gripper == "True":
                if self.loc_count==0:
                    if self.detect:
                        if not self.isDetected:
                            self.set_point[2]+=0.1
                            self.goal_point[2]+=0.1

                        else:
                            self.stopDetection=True
                            self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
                            self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767
                            self.distance()
                            self.path_plan()
                            if -0.2 < self.err_x_m < 0.2 and -0.2 < self.err_y_m < 0.2 :
                                self.set_point[2] = self.alt_setpoint[self.loc_count]+0.35
                                if abs(self.set_point[2]-self.curr_point[2]) < 0.1:
                                    self.gripper_srv(False)
                                    self.loc_count=1
                                    self.set_point[0],self.set_point[1] = self.goal_point[0],self.goal_point[1]
                                    self.set_point[2]=max(self.set_point[2],self.goal_point[2]+self.fly_hieght)
                                    self.goal_point=self.C1   
                                    self.detect = False
                                    self.stopDetection=False
                                    self.gripper = False
                                    while i <100 :
                                        i+= 1
                                    i = 0    
                                    self.drone_state=1
                    elif abs(self.goal_point[0]-self.curr_point[0])<0.00000500 and abs(self.goal_point[1]-self.curr_point[1])<0.00000500:
                        self.altitude_pub.publish(self.alt_setpoint[self.loc_count])
                        if(abs(self.set_point[2]-self.curr_point[2])<0.1 ):
                            self.detect = True                   
                    else:
                        self.path_plan()

                elif self.loc_count==1:
                    if self.detect == True :
                        if not self.isDetected:
                            self.set_point[2]+=0.1
                            self.goal_point[2]+=0.1
                        else:
                            self.stopDetection=True
                            self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
                            self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767
                            self.distance()
                            self.path_plan()                            
                            if -0.2 < self.err_x_m < 0.2 and -0.2 < self.err_y_m < 0.2 :
                                self.set_point[2] = self.alt_setpoint[self.loc_count]+0.35
                                if abs(self.set_point[2]-self.curr_point[2]) < 0.1:                                
                                    self.gripper_srv(False)
                                    self.loc_count=2
                                    self.set_point[0],self.set_point[1] = self.goal_point[0],self.goal_point[1]
                                    self.set_point[2]=max(self.set_point[2],self.alt_setpoint[self.loc_count]+self.fly_hieght)
                                    self.goal_point=self.B2   
                                    self.detect = False
                                    self.stopDetection=False
                                    self.gripper = False
                                    while i <100 :
                                        i+= 1
                                    i = 0
                                    self.drone_state=1
                    elif abs(self.goal_point[0]-self.curr_point[0])<0.000050 and abs(self.goal_point[1]-self.curr_point[1])<0.0000050:
                        rospy.logwarn("Kya hui")
                        self.altitude_pub.publish(self.alt_setpoint[self.loc_count])
                        if(abs(self.set_point[2]-self.curr_point[2])<0.1 ):
                            self.detect = True
                    else:
                        rospy.logwarn("O Bhai")
                        self.path_plan()

                elif self.loc_count==2:
                    rospy.loginfo("point 3")
                    if self.detect == True :
                        if not self.isDetected:
                            self.set_point[2]+=0.1
                            self.goal_point[2]+=0.1
                        else:
                            self.stopDetection=True
                            self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
                            self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767
                            self.distance()
                            self.path_plan()                            
                            if -0.2 < self.err_x_m < 0.2 and -0.2 < self.err_y_m < 0.2 :
                                self.set_point[2] = self.alt_setpoint[self.loc_count]+0.35
                                if abs(self.set_point[2]-self.curr_point[2]) < 0.1:                                
                                    self.gripper_srv(False)
                                    self.set_point[0],self.set_point[1] = self.goal_point[0],self.goal_point[1]
                                    self.set_point[2]=max(self.set_point[2],self.alt_setpoint[2]+self.fly_hieght)
                                    self.goal_point=self.initial_point
                                    self.detect = False
                                    self.stopDetection=False
                                    self.gripper = False
                                    while i <200 :
                                        i+= 1
                                    i = 0                                    
                                    self.drone_state=1
                    elif abs(self.goal_point[0]-self.curr_point[0])<0.000004517/2 and abs(self.goal_point[1]-self.curr_point[1])<0.0000047487/2:
                        self.altitude_pub.publish(self.alt_setpoint[self.loc_count])
                        if(abs(self.set_point[2]-self.curr_point[2])<0.1 ):
                            self.detect = True
                    else:
                        self.path_plan()


                else:
                    rospy.signal_shutdown("Bye")
            else:
                if abs(self.goal_point[0]-self.curr_point[0])<0.0002500 and abs(self.goal_point[1]-self.curr_point[1])<0.0002000:
                    self.set_point[2]=self.goal_point[2]+self.fly_hieght                    
                if abs(self.goal_point[0]-self.curr_point[0])<0.0000800 and abs(self.goal_point[1]-self.curr_point[1])<0.0000800:
                    self.set_point[0]=self.goal_point[0]
                    self.set_point[1]=self.goal_point[1]

                    if abs(self.goal_point[0]-self.curr_point[0])<0.000002217 and abs(self.goal_point[1]-self.curr_point[1])<0.00000228700:
                        self.change(3)
                else:
                    self.path_plan()
        elif self.drone_state==3:##land
            if abs(self.error[2])<0.01:
                self.change(4)   
        elif self.drone_state==4: ##load unload
            if not self.isLoaded:
                self.check_pick_gripper() ##change the goal point in gripper
                self.drone_state=1
            else:
                rospy.signal_shutdown("reached")


    def check_pick_gripper(self):
        # print(self.gripper)
        while self.gripper==False:
            continue
        res=GripperResponse()
        
        while not res.result:
            res = self.gripper_srv(True)
        self.isLoaded=True

    def wall_foll_waypoint_gen(self):
        if 1<self.obs_dist[self.regions["front"]]<5 and 1<self.obs_dist[self.regions["back"]]<5 and self.obs_dist[self.regions["right"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]
        elif 1<self.obs_dist[self.regions["front"]]<5 and 1<self.obs_dist[self.regions["right"]]<5 and self.obs_dist[self.regions["back"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]-self.dy/1.25
        elif 1<self.obs_dist[self.regions["back"]]<5 and 1<self.obs_dist[self.regions["right"]]<5 and self.obs_dist[self.regions["front"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]+self.dy/1.25

        elif 1<self.obs_dist[self.regions["back"]]<5 and self.obs_dist[self.regions["right"]]>5 and self.obs_dist[self.regions["front"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True    
            self.sensor = "B"
            if abs(self.dx) > abs(self.dy) :
                self.set_point[0]=self.prev_setpoint[0]+self.dx*2
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            else :
                self.set_point[0]=self.prev_setpoint[0]+self.dy*6
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
        elif (1<self.obs_dist[self.regions["front"]]<5 or 1<self.obs_dist[self.regions["top"]]<5) and self.obs_dist[self.regions["right"]]>5 and self.obs_dist[self.regions["back"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True
            self.sensor = "F"
            if abs(self.dx) > abs(self.dy) :
                self.set_point[0]=self.prev_setpoint[0]+self.dx*2
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            else :
                self.set_point[0]=self.prev_setpoint[0]+self.dy*6
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25

        elif 1<self.obs_dist[self.regions["right"]]<7 and self.obs_dist[self.regions["front"]]>7 and self.obs_dist[self.regions["back"]]>7 and self.obs_dist[self.regions["left"]]>7:
            self.check = True
            self.sensor = "R"
            if abs(self.dx) > abs(self.dy) :
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]+self.dx*2
            else:
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]+self.dy*6   
        elif self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["front"]]>7 and self.obs_dist[self.regions["back"]]>7 and 1<self.obs_dist[self.regions["left"]]<7:
            self.check = True
            #######################################################
            self.sensor = "L"
            if abs(self.dx) > abs(self.dy) :
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]-self.dx*2
            else:
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]-self.dy*6   
        elif self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["front"]]>7 and self.obs_dist[self.regions["back"]]>7 and self.obs_dist[self.regions["left"]]>7:
            self.set_point[0] = self.prev_setpoint[0] + self.dx
            self.set_point[1] = self.prev_setpoint[1] + self.dy            

    def sensor_reading(self):
        if self.sensor == "F" and self.dx>0:
            self.set_point[0]=self.prev_setpoint[0] + 12/110692.0702932625                       
        if self.sensor == "B":
            self.set_point[0]=self.prev_setpoint[0] - 12/110692.0702932625
            # self.set_point[1] = self.prev_setpoint[1] + 4/105292.0089353767                        
        if self.sensor == "L":
            # self.set_point[0]=self.prev_setpoint[0] - 12/110692.0702932625
            self.set_point[1] = self.prev_setpoint[1] + 5/105292.0089353767                        
        if self.sensor == "R":
            # self.set_point[0]=self.prev_setpoint[0] + 12/110692.0702932625
            self.set_point[1] = self.prev_setpoint[1] + 5/105292.0089353767

    
    def path_plan(self):
        if ((self.obs_dist[0]<7 and self.obs_dist[0]>1) or (self.obs_dist[1]<7 and self.obs_dist[1]>1) or (self.obs_dist[2]<7 and self.obs_dist[2]>1) or (self.obs_dist[3]<7 and self.obs_dist[3]>1)):
            self.wall_foll_waypoint_gen()
            self.prev_setpoint[0] = self.set_point[0]
            self.prev_setpoint[1] = self.set_point[1]             
        else:
            if self.check == True:
                self.sensor_reading()
                if abs(self.set_point[1]-self.curr_point[1]) < 0.000003000 :
                    self.prev_setpoint[0] = self.set_point[0]
                    self.prev_setpoint[1] = self.set_point[1]                     
                    self.distance()
                    print(self.dist)
                    self.check = False
                self.pid()
            else :
                self.set_point[0] = self.prev_setpoint[0] + self.dx
                self.set_point[1] = self.prev_setpoint[1] + self.dy

                self.prev_setpoint[0] = self.set_point[0]
                self.prev_setpoint[1] = self.set_point[1] 




        
        

# main driver code
if __name__ == "__main__":
    e_drone = edrone()
    while not rospy.is_shutdown():
        e_drone.handler()
        e_drone.pid()
        e_drone.r.sleep()