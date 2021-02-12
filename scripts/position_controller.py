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


#Creating Class For implementing given task
class edrone():
    # constructor
    #default object constructor-> gets called everytime when an instance is initialzed
    def __init__(self):
        #initiasing ros node with name position_controller
        rospy.init_node('position_controller')

        # initialising set points as passed from parameters
        self.goal_point = [18.9999999966,72.0000000083,11.44099755545]
        #initialising current points
        self.curr_point = [18.9999999966,72.0000000083,8.44099755545]
        self.set_point = [18.9999999966,72.0000000083,11.44099755545]
        self.initial_point=[18.9999999966,72.0000000083,8.44099755545]


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
        self.obs_dist = [0.0,0.0,0.0,0.0,0.0]
        self.qrcode = [0.0,0.0,0.0]

        self.sample_time = 0.06

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
        self.fly_hieght=4.0000
        self.gripper=False
        self.dist = 0.0
        self.t = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.prev_setpoint = [18.9992411381,71.9998195495,16.6600228704]
        self.r=rospy.Rate(1/self.sample_time)

        self.isDetected=False
        self.stopDetection=False


        self.grid_loc={'lat':[18.9999864489,18.9999864489+0.000013552 ,18.9999864489],'long':[71.9999430161,71.9999430161+0.000014245,71.9999430161+0.000014245+0.000014245],'alt':[8.44099749139,8.44099749139,8.44099749139]}
        self.dest_loc={'lat':[18.9993676146,19.0007030405,19.0004681325],'long':[71.9999999999,71.9999429002,72.0000949773],'alt':[10.654960,22.1600026799,16.660019864]}
        self.curr_index=0

        self.TAKEOFF=False
        self.PATH_PLAN=False
        self.LAND=False
        self.TAKEOFF2=False
        self.PATH_PLAN2=False
        self.DETECT_MARKER=False
        self.LAND2=False

        self.check=False
        self.sensor = ""
        self.err_x_m=0
        self.err_y_m=0
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
        # rospy.Subscriber('/qrscan', qr_code, self.qr_callback)
        rospy.Subscriber("/edrone/marker_data",MarkerData, self.err_xm_callback)
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
        self.err_x_m = msg.err_x_m
        self.err_y_m = msg.err_y_m
    def range_finder_top_callback(self, msg):
        self.obs_dist[self.regions["front"]] = msg.ranges[0]
        self.obs_dist[self.regions["right"]] = msg.ranges[1]
        self.obs_dist[self.regions["back"]] = msg.ranges[2]
        self.obs_dist[self.regions["left"]] = msg.ranges[3]
        self.obs_dist[self.regions["top"]] = msg.ranges[4]
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
        # self.dist = math.sqrt(math.pow((goal_longitude - initial_longitude) , 2) + math.pow((goal_latitude - initial_latitude) , 2))
        # self.t = self.dist/100
        self.dx = (self.goal_point[0] - self.curr_point[0])/100
        self.dy = (self.goal_point[1] - self.curr_point[1])/100
        print(self.dx*100000,self.dy*100000)
        # if 0<self.dx<0.000005:self.dx=0.000005
        # if 0<self.dy<0.000005:self.dy=0.000005
        # if 0>self.dx>-0.000005:self.dx=-0.000005
        # if 0>self.dy>-0.000005:self.dy=-0.000005
        # if self.dx>0.000009:self.dx=0.000009
        # if self.dy>0.000009:self.dy=0.000009
        # if self.dx<0.000009:self.dx=-0.000009
        # if self.dy<0.000009:self.dy=-0.000009
        # print(self.dx,self.dy)

    # def handler(self):
        # if self.drone_state==0:#change to takeoff -> state=1
        #     self.set_point[0]=self.initial_point[0]
        #     self.set_point[1]=self.initial_point[1]
        #     self.set_point[2]=self.initial_point[2]
        #     self.set_point[2]+=self.fly_hieght
        #     self.drone_state=1
        # elif self.drone_state==1:
        #     rospy.loginfo("taking off")
        #     if abs(self.error[2])<0.1:
        #         self.prev_setpoint[0],self.prev_setpoint[1],self.prev_setpoint[2] = self.set_point[0],self.set_point[1],self.set_point[2],
        #         self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.loc_count+1],self.long_setpoint[self.loc_count+1],self.alt_setpoint[self.loc_count+1]
        #         self.drone_state=2
        # elif self.drone_state==2:
        #     self.distance()
        #     if self.loc_count==-1:
        #         self.loc_count+=1
        #         self.set_point[2]=max(self.set_point[2],self.alt_setpoint[self.loc_count]+self.fly_hieght)
        #         self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.loc_count],self.long_setpoint[self.loc_count],self.alt_setpoint[self.loc_count]
        #     elif self.loc_count==0:
        #         rospy.loginfo("point 1")
        #         if self.detect:
        #             if not self.isDetected:
        #                 self.set_point[2]+=0.1
        #                 self.goal_point[2]+=0.1
        #                 print(self.set_point[2])

        #             else:
        #                 self.stopDetection=True
        #                 print("detection success")
        #                 self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
        #                 self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767
        #                 if -1 < self.err_x_m < 1 and -1 < self.err_y_m < 1 :
        #                     self.set_point[2] = self.alt_setpoint[self.loc_count]+5                        
        #                 if -0.2 < self.err_x_m < 0.2 and -0.2 < self.err_y_m < 0.2 :
        #                     self.loc_count=1
        #                     self.set_point[2]=max(self.set_point[2],self.alt_setpoint[self.loc_count]+self.fly_hieght)
        #                     self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.loc_count],self.long_setpoint[self.loc_count],self.alt_setpoint[self.loc_count]   
        #                     self.detect = False
        #                     self.stopDetection=False
        #         elif abs(self.goal_point[0]-self.curr_point[0])<0.00000500 and abs(self.goal_point[1]-self.curr_point[1])<0.00000500:
        #             self.set_point[2]=self.goal_point[2]+1
        #             if(abs(self.set_point[2]-self.curr_point[2])<0.1 ):
        #                 self.detect = True
        # pass



        #     elif self.loc_count==1:
        #         rospy.loginfo("point 2")
        #         if self.detect == True :
        #             if not self.isDetected:
        #                 self.set_point[2]+=0.1
        #                 self.goal_point[2]+=0.1
        #             else:
        #                 self.stopDetection=True
        #                 self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
        #                 self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767
        #                 if -1 < self.err_x_m < 1 and -1 < self.err_y_m < 1 :
        #                     self.set_point[2] = self.alt_setpoint[self.loc_count]+5
        #                 if -0.2 < self.err_x_m < 0.2 and -0.2 < self.err_y_m < 0.2 :
        #                     self.loc_count=2
        #                     self.set_point[2]=max(self.set_point[2],self.alt_setpoint[self.loc_count]+self.fly_hieght)
        #                     self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.loc_count],self.long_setpoint[self.loc_count],self.alt_setpoint[self.loc_count]   
        #                     self.detect = False
        #                     self.stopDetection=False
        #         elif abs(self.goal_point[0]-self.curr_point[0])<0.000050 and abs(self.goal_point[1]-self.curr_point[1])<0.0000050:
        #             self.set_point[2]=self.goal_point[2]+1
        #             if(abs(self.set_point[2]-self.curr_point[2])<0.1 ):
        #                 self.detect = True


        #     elif self.loc_count==2:
        #         rospy.loginfo("point 3")
        #         if self.detect == True :
        #             if not self.isDetected:
        #                 self.set_point[2]+=0.1
        #                 print(self.set_point[2])
        #             else:
        #                 self.stopDetection=True
        #                 self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
        #                 self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767
        #                 if -0.2 < self.err_x_m < 0.2 and -0.2 < self.err_y_m < 0.2 :
        #                     self.drone_state=3
        #                     self.goal_point[2],self.set_point[2]=self.alt_setpoint[2],self.alt_setpoint[2]
        #                     self.detect = False
        #                     self.stopDetection=False
        #         elif abs(self.goal_point[0]-self.curr_point[0])<0.0000050 and abs(self.goal_point[1]-self.curr_point[1])<0.0000050:
        #             self.set_point[2]=self.goal_point[2]+1
        #             rospy.logwarn("reached set point")
        #             rospy.loginfo("setpoint: %f,curr %f",self.set_point[2],self.curr_point[2])
        #             if(abs(self.error[2])<0.2 ):
        #                 self.detect = True
        #                 rospy.logwarn("I have reached altitude,I will now start detection")


        #     else:
        #         rospy.signal_shutdown("Bye")

    def check_pick_gripper(self):
        # print(self.gripper)
        while self.gripper==False:
            rospy.loginfo("gripping unavailable")
            continue
        res=GripperResponse()
        
        while not res.result:
            res = self.gripper_srv(True)
            # rospy.loginfo("Pick up successful")
        if res.result==True: self.isLoaded=True
        if self.isLoaded:return True
    def check_drop_gripper(self):
        self.gripper_srv(False)
        rospy.loginfo("Drop successful")
        self.isLoaded=False
    
    def goto_marker(self):
        if not self.isDetected:
            self.set_point[2]+=0.1
        else:
            self.stopDetection=True
            self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
            self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767

    def is_detection_complete(self):
        print(self.err_x_m,self.err_y_m)
        if -0.2 < self.err_x_m < 0.2 and -0.2 < self.err_y_m < 0.2 and self.isDetected:
            self.stopDetection=False
            return True
        return False
        
    # def wall_foll_waypoint_gen(self):
    #     # if 1<self.obs_dist[self.regions["right"]]<7 and 1<self.obs_dist[self.regions["left"]]<7 and self.obs_dist[self.regions["front"]]>10 and self.obs_dist[self.regions["back"]]>10:
    #     #     self.set_point[0]=self.prev_setpoint[0]-self.dx/6
    #     #     self.set_point[1]=self.prev_setpoint[1]
    #     # elif 1<self.obs_dist[self.regions["right"]]<7 and 1<self.obs_dist[self.regions["front"]]<10 and self.obs_dist[self.regions["left"]]>7 and self.obs_dist[self.regions["back"]]>10:
    #     #     self.set_point[0]=self.prev_setpoint[0]-self.dx/6
    #     #     self.set_point[1]=self.prev_setpoint[1]-self.dy/1.25
    #     #     # rospy.loginfo("R   F")
    #     # elif 1<self.obs_dist[self.regions["left"]]<7 and 1<self.obs_dist[self.regions["front"]]<10 and self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["back"]]>10:
    #     #     self.set_point[0]=self.prev_setpoint[0]-self.dx/6
    #     #     self.set_point[1]=self.prev_setpoint[1]+self.dy/1.25
    #     #     # rospy.loginfo("L   F")

    #     if 1<self.obs_dist[self.regions["left"]]<7 :#and self.obs_dist[self.regions["front"]]>10 and self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["back"]]>10:
    #         self.set_point[0]=self.prev_setpoint[0]+self.dx*1.25
    #         self.set_point[1]=self.prev_setpoint[1]#+self.dy/1.25
    #         rospy.loginfo("   L   ")
    #     if 1<self.obs_dist[self.regions["back"]]<7 :#and self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["left"]]>7 and self.obs_dist[self.regions["back"]]>10:
    #         self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
    #         self.set_point[1]=self.prev_setpoint[1]+self.dy*3
    #         rospy.loginfo("   B   ")
    #     if 1<self.obs_dist[self.regions["right"]]<7 :#and self.obs_dist[self.regions["front"]]>10 and self.obs_dist[self.regions["left"]]>7 and self.obs_dist[self.regions["back"]]>10:
    #         self.set_point[0]=self.prev_setpoint[0]+self.dx*1.25
    #         self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
    #         rospy.loginfo("   R  ")
    #     if 1<self.obs_dist[self.regions["front"]]<7 :#and self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["left"]]>7 and self.obs_dist[self.regions["back"]]>10:
    #         self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
    #         self.set_point[1]=self.prev_setpoint[1]+self.dy*3
    #         rospy.loginfo("   F   ")


    # def path_plan(self):
    #     if ((self.obs_dist[0]<7 and self.obs_dist[0]>1)or (self.obs_dist[1]<7 and self.obs_dist[1]>1)or (self.obs_dist[2]<7 and self.obs_dist[2]>1) or (self.obs_dist[3]<7 and self.obs_dist[3]>1)):
    #         self.wall_foll_waypoint_gen()
    #     else:
    #         self.set_point[0] = self.prev_setpoint[0] + self.dx
    #         self.set_point[1] = self.prev_setpoint[1] + self.dy

    #     self.prev_setpoint[0] = self.set_point[0]
    #     self.prev_setpoint[1] = self.set_point[1] 
    def wall_foll_waypoint_gen(self):
        if 1<self.obs_dist[self.regions["front"]]<7 and 1<self.obs_dist[self.regions["back"]]<7 and self.obs_dist[self.regions["right"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]
        elif 1<self.obs_dist[self.regions["front"]]<7 and 1<self.obs_dist[self.regions["right"]]<7 and self.obs_dist[self.regions["back"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]-self.dy/1.25
            rospy.loginfo("R   F")
        elif 1<self.obs_dist[self.regions["back"]]<7 and 1<self.obs_dist[self.regions["right"]]<7 and self.obs_dist[self.regions["front"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]+self.dy/1.25
            rospy.loginfo("L   F")

        elif 1<self.obs_dist[self.regions["back"]]<5 and self.obs_dist[self.regions["right"]]>5 and self.obs_dist[self.regions["front"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True    
            self.sensor = "B"
            if abs(self.dx) > abs(self.dy) :
                self.set_point[0]=self.prev_setpoint[0]+self.dx
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            else :
                self.set_point[0]=self.prev_setpoint[0]+self.dy*6
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            rospy.loginfo("   B   ")
        elif (1<self.obs_dist[self.regions["front"]]<7 or 1<self.obs_dist[self.regions["top"]]<7) and self.obs_dist[self.regions["right"]]>5 and self.obs_dist[self.regions["back"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True
            self.sensor = "F"
            if abs(self.dx) > abs(self.dy) :
                self.set_point[0]=self.prev_setpoint[0]+self.dx
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            else :
                self.set_point[0]=self.prev_setpoint[0]+self.dy*6
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            rospy.loginfo("   F  ")

        elif 1<self.obs_dist[self.regions["right"]]<7 and self.obs_dist[self.regions["front"]]>5 and self.obs_dist[self.regions["back"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.check = True
            self.sensor = "R"
            if abs(self.dx) > abs(self.dy) :
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]+self.dx/2
            else:
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]+self.dy*6   
            rospy.loginfo("   R   ")
        elif self.obs_dist[self.regions["right"]]>5 and self.obs_dist[self.regions["front"]]>5 and self.obs_dist[self.regions["back"]]>5 and 1<self.obs_dist[self.regions["left"]]<7:
            self.check = True
            self.sensor = "L"
            if abs(self.dx) > abs(self.dy) :
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]+self.dx
            else:
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]+self.dy*6   
            rospy.loginfo("   L   ")
        elif self.obs_dist[self.regions["right"]]>5 and self.obs_dist[self.regions["front"]]>5 and self.obs_dist[self.regions["back"]]>5 and self.obs_dist[self.regions["left"]]>5:
            self.set_point[0] = self.prev_setpoint[0] + self.dx
            self.set_point[1] = self.prev_setpoint[1] + self.dy            

    def sensor_reading(self):
        if self.sensor == "F":
            print("Front")
            self.set_point[0]=self.prev_setpoint[0] + 12/110692.0702932625
            # self.set_point[1] = self.prev_setpoint[1] - 4/105292.0089353767                        
        if self.sensor == "B":
            print("Back")
            self.set_point[0]=self.prev_setpoint[0] - 12/110692.0702932625
            # self.set_point[1] = self.prev_setpoint[1] + 4/105292.0089353767                        
        if self.sensor == "L":
            print("Left")
            # self.set_point[0]=self.prev_setpoint[0] - 12/110692.0702932625
            self.set_point[1] = self.prev_setpoint[1] - 6/105292.0089353767                        
        if self.sensor == "R":
            print("Right")
            # self.set_point[0]=self.prev_setpoint[0] + 12/110692.0702932625
            self.set_point[1] = self.prev_setpoint[1] + 6/105292.0089353767

    
    def path_plan(self):
        if ((self.obs_dist[0]<7 and self.obs_dist[0]>1) or (self.obs_dist[1]<7 and self.obs_dist[1]>1) or (self.obs_dist[2]<7 and self.obs_dist[2]>1) or (self.obs_dist[3]<7 and self.obs_dist[3]>1)):
            self.wall_foll_waypoint_gen()
            self.prev_setpoint[0] = self.set_point[0]
            self.prev_setpoint[1] = self.set_point[1]             
        else:
            if self.check == True:
                # print("Aamchi aai tumchi aai veermata jijabai")
                self.sensor_reading()
                if abs(self.set_point[1]-self.curr_point[1]) < 0.000003000 :
                    # print("Oye Oye")
                    self.prev_setpoint[0] = self.set_point[0]
                    self.prev_setpoint[1] = self.set_point[1]                     
                    self.distance()
                    # print(self.dist)
                    self.check = False
                # self.pid()
            else :
                self.set_point[0] = self.prev_setpoint[0] + self.dx
                self.set_point[1] = self.prev_setpoint[1] + self.dy

                self.prev_setpoint[0] = self.set_point[0]
                self.prev_setpoint[1] = self.set_point[1] 
    def path_plan_comp(self):
        # print(abs(self.goal_point[0]-self.curr_point[0]),abs(self.goal_point[1]-self.curr_point[1]))
        if abs(self.goal_point[0]-self.curr_point[0])<0.00000500 and abs(self.goal_point[1]-self.curr_point[1])<0.00000500:
            return True
        return False
    
    def isAltComplete(self):
        if abs(self.error[2])<0.1:
            return True
        return False

    def handler_2(self):
        if self.curr_index>=3:##3 is number of parcel boxes
            rospy.signal_shutdown("Bye")
            return
        if self.TAKEOFF:
            if self.isAltComplete():
                #setDestination
                rospy.loginfo("Imma goin to fetch the box")
                self.goal_point[0],self.goal_point[1]=self.grid_loc['lat'][self.curr_index],self.grid_loc['long'][self.curr_index]
                # rospy.loginfo("%f ,%f",self.goal_point[0],self.goal_point[1])
                self.PATH_PLAN=True
                self.TAKEOFF=False
        elif self.PATH_PLAN:
            if self.path_plan_comp():
                rospy.loginfo("Imma above the box,will land")
                self.goal_point[2]=self.grid_loc['alt'][self.curr_index]
                self.PATH_PLAN=False
                self.LAND=True
            else:
                self.distance()
                self.path_plan()
        elif self.LAND:
            if self.isAltComplete() and self.check_pick_gripper():
                rospy.loginfo("I reached the box and picked it")
                self.goal_point[2]=max(self.goal_point[2],self.dest_loc['alt'][self.curr_index])+10
                self.TAKEOFF2=True
                self.LAND=False
            else:
                self.distance()
                self.path_plan()
        elif self.TAKEOFF2:
            if self.isAltComplete():
                rospy.loginfo("hieght attained,Imma ready to deliver this parcel")
                self.goal_point[0],self.goal_point[1]=self.dest_loc['lat'][self.curr_index],self.dest_loc['long'][self.curr_index]
                self.PATH_PLAN2=True
                self.TAKEOFF2=False
        elif self.PATH_PLAN2:
            self.distance()
            self.path_plan()
            if self.path_plan_comp() or self.isDetected:
                rospy.loginfo("I've reached the loc,will start detection")
                self.DETECT_MARKER=True
                self.PATH_PLAN2=False
        elif self.DETECT_MARKER:
            self.goto_marker()
            self.distance()
            self.path_plan()
            if self.is_detection_complete():
                rospy.loginfo("marker detected and error minimised")
                self.DETECT_MARKER=False
                self.goal_point[2]=self.dest_loc['alt'][self.curr_index]
                self.LAND2=True
        elif self.LAND2:
            if self.isAltComplete():
                self.check_drop_gripper()
                self.curr_index+=1
                self.LAND2=False
                self.goal_point[2]=max(self.grid_loc['alt'][self.curr_index],self.goal_point[2])+3
                self.TAKEOFF=True

# main driver code
if __name__ == "__main__":
    e_drone = edrone()
    e_drone.TAKEOFF=True
    while not rospy.is_shutdown():
        e_drone.handler_2()
        e_drone.pid()
        e_drone.r.sleep()