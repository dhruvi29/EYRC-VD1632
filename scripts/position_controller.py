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
    # default object constructor-> gets called everytime when an instance is initialzed
    def __init__(self):
        print("init called")
        #initiasing ros node with name position_controller
        rospy.init_node('position_controller')

        # initialising set points as passed from parameters

        #initialising current points
        self.curr_point = [18.9998887906,72.0002184403, 16.757981]
        self.set_point = [18.9998887906,72.0002184403, 16.757981]
        self.initial_point=[18.9998887906,72.0002184403, 16.757981]
        self.goal_point = [18.999837387,72.000156707, 19.757981]
        self.prev_setpoint = [18.9998887906,72.0002184403, 16.757981]

        # self.set_point[2]+=4.00

        #initialsing Roll, Pitch and Yaw 
        self.attitude_cmd = edrone_cmd()
        self.attitude_cmd.rcRoll = 1500.0
        self.attitude_cmd.rcPitch = 1500.0
        self.attitude_cmd.rcYaw = 1500.0
        self.attitude_cmd.rcThrottle = 1500.0
        self.base_value = 1500

        self.Kp = [0.06*4500*176, 1243* 0.06*4500, 1082*0.06]
        self.Ki = [0.0, 0.0, 0.0*0.008]
        self.Kd = [0.3*19000*873, 2102*0.3*19000, 4476*0.3]

        # calculating errors
        self.error = [999.00, 999.0, 999.00]
        self.prev_error = [0.0, 0.0, 0.0]
        self.iterm = [0.0, 0.0, 0.0]
        self.pErr = [0.0, 0.0, 0.0]
        self.output = [0.0, 0.0, 0.0]
        self.dErr = [0.0, 0.0, 0.0]

        #time and sleep
        self.sample_time = 0.06
        self.r=rospy.Rate(1/self.sample_time)

        # sensor reading variables        
        self.obs_dist = [0.0,0.0,0.0,0.0,0.0]
        self.sensor = ""
        self.regions ={
        "front" : 0 ,
        "right" : 1 ,
        "back" : 2,
        "left" : 3,
        "top" : 4
        }

        self.qrcode = [0.0,0.0,0.0]
        self.fly_hieght=4.0000

        # state 1)takeoff 2)fly 3)land 4)gripper check,pick and drop,QR
        self.drone_state=1

        #path planning variables
        self.dist = 0.0
        self.t = 0.0
        self.dist = 0.0
        self.dx = 0.0
        self.dy = 0.0

        #runtime flags
        self.gripper=False
        self.isLoaded=False
        self.grip_check = False

        self.detect = False
        self.isDetected=False
        self.stopDetection=False

        self.ret = False
        self.check=False
        self.loc_count=0

        ## destination location arrays and other variables

        #primary variables
        self.Delivery={'A1':[18.9998102845 ,72.000142461,16.757981],'A2':[18.9998102845 ,72.000156707,16.757981],'A3':[18.9998102845 ,72.000170953,16.757981],'B1':[18.999823836,72.000142461,16.757981],'B2':[18.999823836,72.000156707,16.757981],'B3':[18.999823836,72.000170953,16.757981],'C1':[18.999837387,72.000142461,16.757981],'C3':[18.999837387,72.000170953,16.757981]}
        self.Return= {'X1': [18.9999367615,72.000142461,16.757981],'X2': [18.9999367615,72.000156707,16.757981],'X3': [18.9999367615,72.000170953,16.757981], 'Y1': [18.999950313,72.000142461,16.757981],'Y2': [18.999950313,72.000156707,16.757981],'Y3': [18.999950313,72.000170953,16.757981],'Z1': [18.999963864,72.000142461,16.757981], 'Z2': [18.999963864,72.000156707,16.757981],'Z3': [18.999963864,72.000170953,16.757981]}

        #secondary variables
        self.alt = 0.0 ##range finder bottom altitude
        
        self.lat_setpoint=[]
        self.long_setpoint=[]
        self.alt_setpoint=[]
        
        self.ret_lat_setpoint=[]
        self.ret_long_setpoint=[]
        self.ret_alt_setpoint=[]

        self.point=[]
        self.deliv_grid_point=[]
       

        with open('/home/dhruvi/catkin_ws/src/vitarana_drone/scripts/original.csv', 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                if row[0] == "DELIVERY" :
                    row.pop(0)
                    self.deliv_grid_point.append(row[0])
                    row =row[1].split(";")
                    self.lat_setpoint.append(float(row[0]))
                    self.long_setpoint.append(float(row[1]))
                    self.alt_setpoint.append(float(row[2]))
                if row[0] == "RETURN " :
                    self.point.append(row[2])
                    row.pop(0)
                    row =row[0].split(";")
                    self.ret_lat_setpoint.append(row[0])
                    self.ret_long_setpoint.append(row[1])
                    self.ret_alt_setpoint.append(row[2])


        #--------------------------------------------------------------------------------------------------------------
        # Publishing on Topics - /drone_command , /latitude_error, /longitude_error, /altitude_error

        # The below can be uncommented while tuning latitude, longitude and altitude->
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
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
        # rospy.Subscriber('/qrscan', qr_code, self.qr_callback)
        rospy.Subscriber("/edrone/err_x_m", Float64, self.err_xm_callback)
        rospy.Subscriber("/edrone/err_y_m", Float64, self.err_ym_callback)
        rospy.Subscriber("/isDetected",Bool,self.detection_clbk)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

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
        
    def gripper_callback(self,msg):
        self.gripper=msg.data

    def gps_callback(self, msg):
        self.curr_point[0] = msg.latitude
        self.curr_point[1] = msg.longitude
        self.curr_point[2] = msg.altitude
      
    # the below can be uncommented while tuning latitude, longitude and altitude 
    def range_finder_bottom_callback(self, msg):
        self.alt = msg.ranges[0]

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
        self.dist = math.sqrt(math.pow(110692.0702932625 * (float(self.goal_point[0]) - self.set_point[0]) , 2) + math.pow(105292.0089353767 * (float(self.goal_point[1]) - self.set_point[1]) , 2))
        if 0<self.dist < 5 :
            self.t = self.dist*45
        elif 5 <= self.dist < 10 :
            self.t = self.dist*30
        elif 10 <= self.dist < 40 :
            self.t = self.dist*15
        elif 40 <= self.dist < 80 :
            self.t = self.dist*8
        elif 80 <= self.dist < 120 :
            self.t = self.dist*4
        elif 120 <= self.dist < 160 :
            self.t = self.dist*3
        elif 160 <= self.dist < 200 :
            self.t = self.dist*2
        else :
            self.t = self.dist*1.5
        if self.t == 0:
            self.t = 1            
        self.dx = (self.goal_point[0] - self.set_point[0])/self.t
        self.dy = (self.goal_point[1] - self.set_point[1])/self.t
        print(self.t)

    def nearby_box_return(self):
        global i
        i=0
        n_dist=[]
        while i<len(self.point):
            dist = math.sqrt(math.pow(110692.0702932625 * (float(self.ret_lat_setpoint[i]) - self.curr_point[0]) , 2) + math.pow(105292.0089353767 * (float(self.ret_long_setpoint[i]) - self.curr_point[1]) , 2))
            n_dist.append(dist)
            i+=1
        i = 0
        self.n = n_dist.index(min(n_dist)) 
        n_dist = []  

    def box_delivery(self):
        global i
        i=0
        n_dist=[]
        while i<len(self.point):
            dist = math.sqrt(math.pow(110692.0702932625 * (float(self.lat_setpoint[i]) - self.curr_point[0]) , 2) + math.pow(105292.0089353767 * (float(self.long_setpoint[i]) - self.curr_point[1]) , 2))
            n_dist.append(dist)
            i+=1
        i = 0
        self.loc_count=self.loc_count+1
        if self.loc_count%2==0:
            self.n = n_dist.index(max(n_dist))
        else:
            self.n = n_dist.index(min(n_dist)) 
        n_dist = []  

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
            else :
                self.set_point[0] = self.prev_setpoint[0] + self.dx
                self.set_point[1] = self.prev_setpoint[1] + self.dy

                self.prev_setpoint[0] = self.set_point[0]
                self.prev_setpoint[1] = self.set_point[1]   
    
    def wall_foll_waypoint_gen(self):
        if (1<self.obs_dist[self.regions["front"]]<6 or 1<self.obs_dist[self.regions["back"]]<6) and 1<self.obs_dist[self.regions["right"]]<6  and self.obs_dist[self.regions["left"]]>6:
            self.check = True
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]-self.dy/1.25
            rospy.loginfo("R   F")
        elif (1<self.obs_dist[self.regions["front"]]<6 or 1<self.obs_dist[self.regions["back"]]<6) and 1<self.obs_dist[self.regions["right"]]<6 and self.obs_dist[self.regions["left"]]>6:
            self.check = True
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]+self.dy/1.25
            rospy.loginfo("L   F")


        elif (1<self.obs_dist[self.regions["front"]]<6 or 1<self.obs_dist[self.regions["top"]]<6 or 1<self.obs_dist[self.regions["back"]]<6) and self.obs_dist[self.regions["right"]]>6  and self.obs_dist[self.regions["left"]]>6:
            self.check = True
            self.sensor = "F or B"
            if (self.dx > 0 and self.dy > 0) or (self.dx < 0 and self.dy < 0):
                if abs(self.dx) > abs(self.dy) :
                    self.set_point[0]=self.prev_setpoint[0]+self.dx*6
                    self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
                else :
                    self.set_point[0]=self.prev_setpoint[0]+self.dy*6
                    self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            elif (self.dx < 0 and self.dy > 0) or (self.dx > 0 and self.dy < 0):
                if abs(self.dx) > abs(self.dy) :
                    self.set_point[0]=self.prev_setpoint[0]+self.dx*6
                    self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
                else :
                    self.set_point[0]=self.prev_setpoint[0]-self.dy*6
                    self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25                         
            rospy.loginfo("   F  ")

        elif (1<self.obs_dist[self.regions["right"]]<6 or 1<self.obs_dist[self.regions["left"]]<6) and self.obs_dist[self.regions["front"]]>6 and self.obs_dist[self.regions["back"]]>6 :
            self.check = True
            self.sensor = "L or R"
            if (self.dx > 0 and self.dy > 0) or (self.dx < 0 and self.dy < 0):
                if abs(self.dx) > abs(self.dy) :
                    self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                    self.set_point[1]=self.prev_setpoint[1]+self.dx*6
                else:
                    self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                    self.set_point[1]=self.prev_setpoint[1]+self.dy*6   
            elif (self.dx < 0 and self.dy > 0) or (self.dx > 0 and self.dy < 0):
                if abs(self.dx) > abs(self.dy) :
                    self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                    self.set_point[1]=self.prev_setpoint[1]-self.dx*6
                else:
                    self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                    self.set_point[1]=self.prev_setpoint[1]+self.dy*6    
            rospy.loginfo("   R   ")

        elif self.obs_dist[self.regions["right"]]>6 and self.obs_dist[self.regions["front"]]>6 and self.obs_dist[self.regions["back"]]>6 and self.obs_dist[self.regions["left"]]>6:
            self.set_point[0] = self.prev_setpoint[0] + self.dx/50
            self.set_point[1] = self.prev_setpoint[1] + self.dy/50          
    
    def sensor_reading(self):
        if self.sensor == "F or B":
            print("Front or Back")
            if self.dx > 0 :
                self.set_point[0]=self.prev_setpoint[0] + 6/110692.0702932625
            elif self.dx < 0 :
                self.set_point[0]=self.prev_setpoint[0] - 6/110692.0702932625

        if self.sensor == "L or R" :
            print("Left or Right")
            if self.dy > 0 :
                self.set_point[1] = self.prev_setpoint[1] + 6/105292.0089353767                        
            elif self.dy < 0 :
                self.set_point[1] = self.prev_setpoint[1] - 6/105292.0089353767  
    
    def check_pick_gripper(self):
        while self.gripper==False:
            continue
        res=GripperResponse()
        
        while not res.result:
            res = self.gripper_srv(True)
        self.isLoaded=True

    def find_marker(self):
        if not self.isDetected:
            self.set_point[2]+=0.1
            self.goal_point[2]+=0.1

        else:
            self.stopDetection=True
            print("detection success")
            self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
            self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767
            self.distance()
            self.path_plan()
            if -0.1 < self.err_x_m < 0.1 and -0.1 < self.err_y_m < 0.1 :
                self.stopDetection=False
                self.change_state()

            elif -0.3 < self.err_x_m < 0.3 and -0.3 < self.err_y_m < 0.3 :
                self.set_point[2] = self.alt_setpoint[self.loc_count]+0.4


    def load(self):
        while self.gripper==False:
            continue
        res=GripperResponse()
        
        while not res.result:
            res = self.gripper_srv(True)
        if res.result:
            self.isLoaded=True
            self.change_state()
        

    def unload(self):
        self.gripper_srv(False)
        self.isLoaded=False
        self.change_state()

    def change_state(self):
        ''' state 1=take off, state 2=fly state 3=land state 4=gripper check,pick drop '''
        if self.drone_state==1:
            self.distance()
            self.drone_state=2
            self.set_point[2]=max(self.set_point[2],self.goal_point[2]+4)
        elif self.drone_state==2:
            self.drone_state=3
            self.error[2]=1
            self.set_point[2]=self.goal_point[2]
        elif self.drone_state==3:            
            self.drone_state=4
        else:
            self.drone_state=1
            self.set_point[2]=self.set_point[2]+4

    def state1(self):
        print("error",self.error[2])
        if self.error[2]<0.01:
            if self.ret:
                if self.isLoaded:
                    print("return",self.point[self.n])
                    self.goal_point=self.Return[self.point[self.n]]
                    self.point.pop(self.n)
                else:
                    self.nearby_box_return()
                    self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.ret_lat_setpoint[self.n],self.ret_long_setpoint[self.n],self.ret_alt_setpoint[self.n]
                    self.ret_lat_setpoint.pop(self.n)
                    self.ret_long_setpoint.pop(self.n)
                    self.ret_alt_setpoint.pop(self.n)
            else:
                if self.isLoaded:
                    self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.n],self.long_setpoint[self.n],self.alt_setpoint[self.n]
                    self.lat_setpoint.pop(self.n)
                    self.long_setpoint.pop(self.n)
                    self.alt_setpoint.pop(self.n)
                else:
                    self.box_delivery()
                    self.goal_point=self.Delivery[self.deliv_grid_point[self.n]]
                    print("pickup",self.deliv_grid_point[self.n])
                    self.deliv_grid_point.pop(self.n)
            self.change_state()
    
    def state2(self):
        if not self.ret and self.isLoaded and abs(self.goal_point[0]-self.curr_point[0])<0.00000500 and abs(self.goal_point[1]-self.curr_point[1])<0.00000500:
            
            self.find_marker()
        elif abs(self.goal_point[0]-self.curr_point[0])<0.000100000 and abs(self.goal_point[1]-self.curr_point[1])<0.000100000:
            print("Hello")
            self.set_point[0]=self.goal_point[0]
            self.set_point[1]=self.goal_point[1]
            # self.change_state()
            if abs(self.goal_point[0]-self.curr_point[0])<0.000000600 and abs(self.goal_point[1]-self.curr_point[1])<0.000000600:
                self.change_state()
        else:        
            self.path_plan()

    def state3(self):
        if self.error[2]<0.01:
            self.change_state()

    def state4(self):
        if self.isLoaded:
            self.unload()
            self.ret=not self.ret
        else:
            self.load()

    def handler(self):
        if self.drone_state==1:
            # print(self.drone_state)
            self.state1()
        elif self.drone_state==2:
            # print(self.drone_state)
            self.state2()
        elif self.drone_state==3:##land
            # print(self.drone_state)
            self.state3()
        elif self.drone_state==4: ##load unload
            # print(self.drone_state)
            self.state4()

# main driver code
if __name__ == "__main__":
    e_drone = edrone()
    while not rospy.is_shutdown():
        e_drone.handler()
        e_drone.pid()
        e_drone.r.sleep()