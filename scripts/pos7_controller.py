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
i,n = 0,0

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

        #initialsing Roll, Pitch and Yaw 
        self.attitude_cmd = edrone_cmd()
        self.attitude_cmd.rcRoll = 1500.0
        self.attitude_cmd.rcPitch = 1500.0
        self.attitude_cmd.rcYaw = 1500.0
        self.attitude_cmd.rcThrottle = 1500.0
        self.base_value = 1500

        # setting PID constants based on tuning
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        self.Kp = [0.06*5100*176, 1243* 0.06*5100, 1500*0.06]
        self.Ki = [0.0, 0.0, 0.0*0.008]
        self.Kd = [0.3*20000*873, 2102*0.3*20000, 5500*0.3]
=======
        self.Kp = [0.06*5000*176, 1243* 0.06*5000, 1500*0.06]
        self.Ki = [0.0, 0.0, 0.0*0.008]
        self.Kd = [0.3*21000*873, 2102*0.3*21000, 5300*0.3]
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        self.Kp = [0.06*5000*176, 1243* 0.06*5000, 1500*0.06]
        self.Ki = [0.0, 0.0, 0.0*0.008]
        self.Kd = [0.3*21000*873, 2102*0.3*21000, 5300*0.3]
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        self.Kp = [0.06*5000*176, 1243* 0.06*5000, 1500*0.06]
        self.Ki = [0.0, 0.0, 0.0*0.008]
        self.Kd = [0.3*21000*873, 2102*0.3*21000, 5300*0.3]
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a

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
        self.n = 0
        self.isLoaded=False
        self.fly_hieght=4.0000
        self.gripper=False
        self.dist = 0.0
        self.t = 0.0
        self.dist = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.prev_setpoint = [18.9998887906,72.0002184403, 16.757981]
        self.r=rospy.Rate(1/self.sample_time)

        self.grip_check = False
        self.detect = False
        self.ret = False
        self.alt = 0.0
        self.lat_setpoint=[]
        self.long_setpoint=[]
        self.alt_setpoint=[]
        self.ret_lat_setpoint=[]
        self.ret_long_setpoint=[]
        self.ret_alt_setpoint=[]
        self.point=[]
        self.delivery = []
        self.n_dist=[]
        self.grid = []
        self.Delivery= {'A1': [18.9998102845 ,72.000142461,16.757981],'A2': [18.9998102845 ,72.000156707,16.757981],'A3': [18.9998102845 ,72.000170953,16.757981], 'B1': [18.999823836,72.000142461,16.757981],'B2': [18.999823836,72.000156707,16.757981],'B3': [18.999823836,72.000170953,16.757981],'C1': [18.999837387,72.000142461,16.757981], 'C2': [18.999837387,72.000156707, 16.757981],'C3': [18.999837387,72.000170953,16.757981]}

        self.Return= {'X1 ': [18.9999367615,72.000142461,16.757981],'X2 ': [18.9999367615,72.000156707,16.757981],'X3': [18.9999367615,72.000170953,16.757981], 'Y1 ': [18.999950313,72.000142461,16.757981],'Y2 ': [18.999950313,72.000156707,16.757981],'Y3 ': [18.999950313,72.000170953,16.757981],'Z1 ': [18.999963864,72.000142461,16.757981], 'Z2': [18.999963864,72.000156707,16.757981],'Z3 ': [18.999963864,72.000170953,16.757981]}
        self.loc_count=0
        self.isDetected=False
        self.stopDetection=False
        self.check=False

        # Getting delivery and return coordinates from csv file and storing it in lists
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        with open('/home/karthikswami/catkin_ws/src/vitarana_drone/scripts/manifest.csv', 'r') as file:
=======
        with open('/home/dhairya/catkin_ws/src/vitarana_drone/scripts/manifest.csv', 'r') as file:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        with open('/home/dhairya/catkin_ws/src/vitarana_drone/scripts/manifest.csv', 'r') as file:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        with open('/home/dhairya/catkin_ws/src/vitarana_drone/scripts/manifest.csv', 'r') as file:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
            reader = csv.reader(file)
            for row in reader:
                if row[0] == "DELIVERY" :
                    row.pop(0)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    # if row[0]!="B2":
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
                    self.delivery.append(row[0])
                    row =row[1].split(";")
                    self.lat_setpoint.append(float(row[0]))
                    self.long_setpoint.append(float(row[1]))
                    self.alt_setpoint.append(float(row[2]))
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    # else:
                    #     continue
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a

                if row[0] == "RETURN " :
                    self.point.append(row[2])
                    row.pop(0)
                    row =row[0].split(";")
                    self.ret_lat_setpoint.append(row[0])
                    self.ret_long_setpoint.append(row[1])
                    self.ret_alt_setpoint.append(row[2])

        self.seq_delivery = {}
        self.seq_return = {}
        self.seq_delivery_dist = {}
        self.seq_return_dist = {}
        self.sorted_delivery_index = []
        self.sorted_return_index = []

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        with open('/home/karthikswami/catkin_ws/src/vitarana_drone/scripts/manifest.csv', 'r') as file:
=======
        with open('/home/dhairya/catkin_ws/src/vitarana_drone/scripts/manifest.csv', 'r') as file:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        with open('/home/dhairya/catkin_ws/src/vitarana_drone/scripts/manifest.csv', 'r') as file:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        with open('/home/dhairya/catkin_ws/src/vitarana_drone/scripts/manifest.csv', 'r') as file:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
            reader = csv.reader(file)
            for row in reader:
                if row[0]=="DELIVERY":
                    self.seq_delivery[row[1]] = [float(r) for r in row[2].split(";")]

                elif row[0]=="RETURN ":    
                    self.seq_return[row[2]] = [float(r) for r in row[1].split(";")]

        for (key, val) in self.seq_delivery.items():
            m = math.sqrt(math.pow(110692.0702932625 * (val[0] - self.Delivery[key][0]) , 2) + math.pow(105292.0089353767 * (val[1] - self.Delivery[key][1]) , 2))
            self.seq_delivery_dist[key] = m
        sorted_delivery = sorted(self.seq_delivery_dist.values(),reverse=True)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        for n1 in sorted_delivery:
            for keys in self.seq_delivery_dist.keys():
                if  self.seq_delivery_dist[keys] == n1:
=======
        for n in sorted_delivery:
            for keys in self.seq_delivery_dist.keys():
                if  self.seq_delivery_dist[keys] == n:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        for n in sorted_delivery:
            for keys in self.seq_delivery_dist.keys():
                if  self.seq_delivery_dist[keys] == n:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        for n in sorted_delivery:
            for keys in self.seq_delivery_dist.keys():
                if  self.seq_delivery_dist[keys] == n:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
                    self.sorted_delivery_index.append(keys)
                    break 

        m = []
        for key in self.sorted_delivery_index:
            for key2, val2 in self.seq_return.items():  
                m.append(math.sqrt(math.pow(110692.0702932625 * (self.seq_delivery[key][0] - val2[0]) , 2) + math.pow(105292.0089353767 * (self.seq_delivery[key][1] - val2[1]) , 2)))
                k = m.index(min(m),0,)
            self.seq_return_dist[self.seq_return.keys()[k]] = min(m)
            m = []
        sorted_return = sorted(self.seq_return_dist.values())
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        for n1 in sorted_return:
            for keys in self.seq_return_dist.keys():
                if  self.seq_return_dist[keys] == n1:
                    self.sorted_return_index.append(keys)
                    break 
        
        with open('/home/karthikswami/catkin_ws/src/vitarana_drone/scripts/sequenced_manifest.csv', 'w') as file:
            writer = csv.writer(file)
            for c in range(len(self.sorted_delivery_index)-1):
                writer.writerow(["DELIVERY",self.sorted_delivery_index[c],self.seq_delivery[self.sorted_delivery_index[c]][0],self.seq_delivery[self.sorted_delivery_index[c]][1],self.seq_delivery[self.sorted_delivery_index[c]][2]])
                print("DELIVERY: ", self.sorted_delivery_index[c])
                writer.writerow(["RETURN",self.seq_return[self.sorted_return_index[c]][0],self.seq_return[self.sorted_return_index[c]][1],self.seq_return[self.sorted_return_index[c]][2],self.sorted_return_index[c]])
                print("RETURN",self.sorted_return_index[c])
=======
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
        for n in sorted_return:
            for keys in self.seq_return_dist.keys():
                if  self.seq_return_dist[keys] == n:
                    self.sorted_return_index.append(keys)
                    break 
        
        with open('/home/dhairya/catkin_ws/src/vitarana_drone/scripts/sequenced_manifest.csv', 'w') as file:
            writer = csv.writer(file)
            for i in range(len(self.sorted_delivery_index)-1):
                writer.writerow(["DELIVERY",self.sorted_delivery_index[i],self.seq_delivery[self.sorted_delivery_index[i]][0],self.seq_delivery[self.sorted_delivery_index[i]][1],self.seq_delivery[self.sorted_delivery_index[i]][2]])
                print("DELIVERY: ", self.sorted_delivery_index[i])
                writer.writerow(["RETURN",self.seq_return[self.sorted_return_index[i]][0],self.seq_return[self.sorted_return_index[i]][1],self.seq_return[self.sorted_return_index[i]][2],self.sorted_return_index[i]])
                print("RETURN",self.sorted_return_index[i])
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
        #--------------------------------------------------------------------------------------------------------------
        # Publishing on Topics - /drone_command , /latitude_error, /longitude_error, /altitude_error
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # The below can be uncommented while tuning latitude, longitude and altitude->
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        self.latitude_error_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        self.longitude_error_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)
        self.altitude_pub = rospy.Publisher('/altitude', Float32, queue_size=1)

        # Subscriptions on topics /edrone/gps, /pid_tuning_pitch,. /pid_tuning_roll, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid) 
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid) 
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
=======
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
        # self.latitude_error_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        # self.longitude_error_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        # self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)
        # self.altitude_pub = rospy.Publisher('/altitude', Float32, queue_size=1)

        # Subscriptions on topics /edrone/gps, /pid_tuning_pitch,. /pid_tuning_roll, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid) 
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid) 
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
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
    # def qr_callback(self,msg):
    #     #print('in qr callback')
    #     self.qrcode[0]= msg.latitude
    #     self.qrcode[1]= msg.longitude
    #     self.qrcode[2]= msg.altitude
    #     rospy.loginfo('QR: %f %f %f',self.qrcode[0],self.qrcode[1],self.qrcode[2])

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    def latitude_set_pid(self, msg):
        self.Kp[0] = msg.Kp * (200)
        self.Ki[0] = msg.Ki * 10
        self.Kd[0] = msg.Kd * 10000

    def longitude_set_pid(self, msg):
        self.Kp[1] = msg.Kp * (200)
        self.Ki[1] = msg.Ki * 10
        self.Kd[1] = msg.Kd * 10000

    def altitude_set_pid(self, msg):
        self.Kp[2] = msg.Kp * 0.2
        self.Ki[2] = msg.Ki * 0.008
        self.Kd[2] = msg.Kd * 0.3
=======
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
    # def latitude_set_pid(self, msg):
    #     self.Kp[0] = msg.Kp * (100)
    #     self.Ki[0] = msg.Ki * 10
    #     self.Kd[0] = msg.Kd * 1000

    # def longitude_set_pid(self, msg):
    #     self.Kp[1] = msg.Kp * (100)
    #     self.Ki[1] = msg.Ki * 10
    #     self.Kd[1] = msg.Kd * 1000

    # def altitude_set_pid(self, msg):
    #     self.Kp[2] = msg.Kp * 0.2
    #     self.Ki[2] = msg.Ki * 0.008
    #     self.Kd[2] = msg.Kd * 0.3
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
 
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        self.latitude_error_pub.publish(self.error[0])
        self.longitude_error_pub.publish(self.error[1])
        self.altitude_error_pub.publish(self.error[2])
=======
        # self.latitude_error_pub.publish(self.error[0]*1000000)
        # self.longitude_error_pub.publish(self.error[1])
        # self.altitude_error_pub.publish(self.error[2])
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        # self.latitude_error_pub.publish(self.error[0]*1000000)
        # self.longitude_error_pub.publish(self.error[1])
        # self.altitude_error_pub.publish(self.error[2])
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        # self.latitude_error_pub.publish(self.error[0]*1000000)
        # self.longitude_error_pub.publish(self.error[1])
        # self.altitude_error_pub.publish(self.error[2])
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a

    # function to give setpoints with respect to the distance between initial and final points
    def distance(self):
        self.dist = math.sqrt(math.pow(110692.0702932625 * (self.goal_point[0] - self.set_point[0]) , 2) + math.pow(105292.0089353767 * (self.goal_point[1] - self.set_point[1]) , 2))
        if 0<self.dist < 5 :
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            self.t = self.dist*50      
        elif 5 <= self.dist < 10 :
            self.t = self.dist*15
        elif 10 <= self.dist < 40 :
            self.t = self.dist*10
        elif 40 <= self.dist < 80 :
            self.t = self.dist*7
        elif 80 <= self.dist < 120 :
            self.t = self.dist*4.5
        elif 120 <= self.dist < 160 :
            self.t = self.dist*4.5
        elif 160 <= self.dist < 200 :
            self.t = self.dist*4
        else :
            self.t = self.dist*3.5
=======
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
            self.t = self.dist*70       
        elif 5 <= self.dist < 10 :
            self.t = self.dist*20
        elif 10 <= self.dist < 40 :
            self.t = self.dist*10
        elif 40 <= self.dist < 80 :
            self.t = self.dist*8
        elif 80 <= self.dist < 120 :
            self.t = self.dist*6
        elif 120 <= self.dist < 160 :
            self.t = self.dist*5
        elif 160 <= self.dist < 200 :
            self.t = self.dist*4.5
        else :
            self.t = self.dist*4
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
        if self.t == 0:
            self.t = 1            
        self.dx = (self.goal_point[0] - self.set_point[0])/self.t
        self.dy = (self.goal_point[1] - self.set_point[1])/self.t
        print(self.t)

    def box_delivery(self):
        global i,n
        i=0
        n_dist=[]
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        print(self.delivery)
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
        while i<len(self.delivery):
            dist = math.sqrt(math.pow(110692.0702932625 * (float(self.lat_setpoint[i]) - self.curr_point[0]) , 2) + math.pow(105292.0089353767 * (float(self.long_setpoint[i]) - self.curr_point[1]) , 2))
            n_dist.append(dist)
            i+=1
        i = 0
        # self.loc_count=self.loc_count+1
        # if self.loc_count%2==0:
        n = n_dist.index(max(n_dist))
        self.goal_point = self.Delivery[self.delivery[n]]       
        self.delivery.pop(n)
        # self.lat_setpoint.pop(self.n)
        # self.long_setpoint.pop(self.n)
        # else:
        #     self.n = n_dist.index(min(n_dist)) 
        n_dist = []  

    # function to calculate the distance of the nearest box for return
    def nearby_box(self):
        global i
        while i<len(self.point):
            self.dist = math.sqrt(math.pow(110692.0702932625 * (float(self.ret_lat_setpoint[i]) - self.curr_point[0]) , 2) + math.pow(105292.0089353767 * (float(self.ret_long_setpoint[i]) - self.curr_point[1]) , 2))
            self.n_dist.append(self.dist)
            i+=1
        i = 0
        print(self.n_dist)
        self.n = self.n_dist.index(min(self.n_dist)) 
        print(self.n)
        self.n_dist = []  
        self.goal_point[0],self.goal_point[1],self.goal_point[2] = float(self.ret_lat_setpoint[self.n]),float(self.ret_long_setpoint[self.n]),float(self.ret_alt_setpoint[self.n])+7
        self.grid.append(self.point[self.n])
        # self.point.pop(self.n)
        # self.ret_lat_setpoint.pop(self.n)
        # self.ret_long_setpoint.pop(self.n)         
        self.ret = True

    # Function just to change states and provide goal point and set points
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
            # self.set_point[2]=self.goal_point[2]
            if self.ret == False:
                self.set_point[2]=self.goal_point[2]
            else:
                if self.gripper == "False":    
                    self.set_point[2]=float(self.ret_alt_setpoint[self.n])
                else:
                    self.set_point[2]=self.Return[self.point[self.n]][2]+1

    # function to drop the box on marker/grid respectively
    def box_dropping(self): 
        global i           
        if self.ret == False:
            if self.detect:
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    # if abs(self.goal_point[0]-self.curr_point[0])<0.00003000 and abs(self.goal_point[1]-self.curr_point[1])<0.00003000:
                    #     self.set_point[2] = self.alt_setpoint[n]+7                       
=======
                    # if -1 < self.err_x_m < 1 and -1 < self.err_y_m < 1 :
                    #     self.set_point[2] = self.alt_setpoint[self.loc_count]+5                        
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
                    # if -1 < self.err_x_m < 1 and -1 < self.err_y_m < 1 :
                    #     self.set_point[2] = self.alt_setpoint[self.loc_count]+5                        
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
                    # if -1 < self.err_x_m < 1 and -1 < self.err_y_m < 1 :
                    #     self.set_point[2] = self.alt_setpoint[self.loc_count]+5                        
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
                    if -0.3 < self.err_x_m < 0.3 and -0.3 < self.err_y_m < 0.3 :
                        self.set_point[2] = self.alt_setpoint[n]+0.4
                        # self.set_point[2] = self.set_point[2]-0.1
                        if abs(self.set_point[2]-self.curr_point[2]) < 0.2:
                        # if abs(self.curr_point[2]-self.alt_setpoint[self.loc_count]) < 0.5:
                            self.gripper_srv(False)
                            self.gripper = False
                            self.loc_count+=1
                            self.set_point[0],self.set_point[1] = self.goal_point[0],self.goal_point[1]
                            
                            if self.point == []: 
                                self.set_point[2]=max(self.set_point[2]+2,self.goal_point[2]+2)#self.fly_hieght)
                                self.goal_point=self.initial_point  
                            else:
                                self.set_point[0],self.set_point[1]= self.curr_point[0],self.curr_point[1]
                                # self.set_point[2] = self.alt_setpoint[self.loc_count-1]+2
                                self.nearby_box()
                                self.set_point[2]=max(self.set_point[2],self.goal_point[2])#self.fly_hieght)
                            self.detect = False
                            self.stopDetection=False
                            while i <100 :
                                i+= 1
                            i = 0    
                            self.drone_state=1
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            elif abs(self.goal_point[0]-self.curr_point[0])<0.00001500 and abs(self.goal_point[1]-self.curr_point[1])<0.00001500:
                self.altitude_pub.publish(self.alt_setpoint[n])
                if(abs(self.set_point[2]-self.curr_point[2])<0.3 ):
                    self.detect = True                   
            else:
                if abs(self.goal_point[0]-self.curr_point[0])<0.0003000 and abs(self.goal_point[1]-self.curr_point[1])<0.000300:
                    self.set_point[2]=self.alt_setpoint[n]+12
                if abs(self.goal_point[0]-self.curr_point[0])<0.0001500 and abs(self.goal_point[1]-self.curr_point[1])<0.0001500:
                    self.set_point[0]=self.goal_point[0]
                    self.set_point[1]=self.goal_point[1]    
                else:                  
                    self.path_plan()
        else:
            if abs(self.goal_point[0]-self.curr_point[0])<0.0002500 and abs(self.goal_point[1]-self.curr_point[1])<0.0002500:
=======
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
            elif abs(self.goal_point[0]-self.curr_point[0])<0.00000500 and abs(self.goal_point[1]-self.curr_point[1])<0.00000500:
                self.altitude_pub.publish(self.alt_setpoint[n])
                if(abs(self.set_point[2]-self.curr_point[2])<0.2 ):
                    self.detect = True                   
            else:
                if abs(self.goal_point[0]-self.curr_point[0])<0.00025000 and abs(self.goal_point[1]-self.curr_point[1])<0.0002500:
                    self.set_point[2]=self.alt_setpoint[n]+11    
                self.path_plan()
        else:
            if abs(self.goal_point[0]-self.curr_point[0])<0.0002000 and abs(self.goal_point[1]-self.curr_point[1])<0.0002000:
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
                self.set_point[2]=self.Return[self.point[self.n]][2]+self.fly_hieght                    
            if abs(self.goal_point[0]-self.curr_point[0])<0.0001000 and abs(self.goal_point[1]-self.curr_point[1])<0.0001000:
                self.set_point[0]=self.goal_point[0]
                self.set_point[1]=self.goal_point[1]

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                if abs(self.goal_point[0]-self.curr_point[0])<0.000002000 and abs(self.goal_point[1]-self.curr_point[1])<0.000002000:
=======
                if abs(self.goal_point[0]-self.curr_point[0])<0.000000600 and abs(self.goal_point[1]-self.curr_point[1])<0.000000600:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
                if abs(self.goal_point[0]-self.curr_point[0])<0.000000600 and abs(self.goal_point[1]-self.curr_point[1])<0.000000600:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
                if abs(self.goal_point[0]-self.curr_point[0])<0.000000600 and abs(self.goal_point[1]-self.curr_point[1])<0.000000600:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
                    self.set_point[2] = self.Return[self.point[self.n]][2]+0.3
                    if abs(self.set_point[2]-self.curr_point[2])<0.01:
                        self.ret = False
                        self.gripper_srv(False) 
                        self.lat_setpoint.pop(n)
                        self.long_setpoint.pop(n)                     
                        self.alt_setpoint.pop(n)                                        
                        # self.ret_alt_setpoint.pop(self.n)                         
                        self.gripper = False
                        if self.delivery == []: 
                            self.goal_point=self.initial_point                      
                            self.set_point[2]=max(self.set_point[2],self.alt_setpoint[2])#self.fly_hieght)                            
                        else:
                            self.box_delivery()
                            # self.goal_point=self.Delivery[self.delivery[self.n]] 
                            self.set_point[2]=max(self.set_point[2],self.goal_point[2])#self.fly_hieght)
                        while i <100 :
                            i+= 1
                        i = 0 
                        self.drone_state=1  
            else:
                self.path_plan()

    # Handler for carrying out different states of drone like takeoff, landing,calling funtions for box picking/dropping  
    def handler(self):
        global i
        if self.drone_state==0:#change to takeoff -> state=1
            self.box_delivery()
            self.set_point[2]=self.curr_point[2]
            if self.goal_point[2]>self.curr_point[2]:
                self.set_point[2]= self.fly_hieght+self.goal_point[2]
            else:
                self.set_point[2]+=self.fly_hieght
            self.drone_state=1
        elif self.drone_state==1:
            self.isLoaded = False
            rospy.loginfo("taking off")
            if abs(self.error[2])<0.1:
                self.prev_setpoint[0],self.prev_setpoint[1],self.prev_setpoint[2] = self.set_point[0],self.set_point[1],self.set_point[2]
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                # while self.grip_check == True:
                #     if self.gripper == "True":
                #         if self.ret == False:
                #             self.set_point[0],self.set_point[1],self.set_point[2]=self.curr_point[0],self.curr_point[1],max(self.set_point[2],self.alt_setpoint[n]+15)
                #             # print(self.set_point[2])
                #             self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[n],self.long_setpoint[n],self.alt_setpoint[n]+15
                #             self.grip_check = False
                #             break
                #         else:
                #             self.set_point[0],self.set_point[1],self.set_point[2]=self.curr_point[0],self.curr_point[1],max(self.set_point[2],self.Return[self.point[self.n]][2]+15)
                #             # print(self.set_point[2])
                #             self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.Return[self.point[self.n]][0],self.Return[self.point[self.n]][1],self.Return[self.point[self.n]][2]+15
                #             self.grip_check = False
                #             break                            
                #     else:
                #         continue          
=======
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
                while self.grip_check == True:
                    if self.gripper == "True":
                        if self.ret == False:
                            self.set_point[0],self.set_point[1],self.set_point[2]=self.curr_point[0],self.curr_point[1],max(self.set_point[2],self.alt_setpoint[n]+self.fly_hieght)
                            # print(self.set_point[2])
                            self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[n],self.long_setpoint[n],self.alt_setpoint[n]+15
                            self.grip_check = False
                            break
                        else:
                            self.set_point[0],self.set_point[1],self.set_point[2]=self.curr_point[0],self.curr_point[1],max(self.set_point[2],self.alt_setpoint[n]+self.fly_hieght)
                            # print(self.set_point[2])
                            self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.Return[self.point[self.n]][0],self.Return[self.point[self.n]][1],self.Return[self.point[self.n]][2]+15
                            self.grip_check = False
                            break                            
                    else:
                        continue          
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
                self.distance()                
                self.change(2)
        elif self.drone_state==2:
            if self.gripper == "True":
                self.box_dropping()
            else:
                if abs(self.goal_point[0]-self.curr_point[0])<0.0002500 and abs(self.goal_point[1]-self.curr_point[1])<0.0002500:
                    if self.ret == False:
                        self.set_point[2]=self.goal_point[2]+self.fly_hieght    
                    else:                    
                        self.set_point[2]=float(self.ret_alt_setpoint[self.n])+self.fly_hieght                    
                if abs(self.goal_point[0]-self.curr_point[0])<0.0001500 and abs(self.goal_point[1]-self.curr_point[1])<0.0001500:
                    self.set_point[0]=self.goal_point[0]
                    self.set_point[1]=self.goal_point[1]

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    if abs(self.goal_point[0]-self.curr_point[0])<0.000001500 and abs(self.goal_point[1]-self.curr_point[1])<0.000001500:
=======
                    if abs(self.goal_point[0]-self.curr_point[0])<0.000000600 and abs(self.goal_point[1]-self.curr_point[1])<0.000000600:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
                    if abs(self.goal_point[0]-self.curr_point[0])<0.000000600 and abs(self.goal_point[1]-self.curr_point[1])<0.000000600:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
                    if abs(self.goal_point[0]-self.curr_point[0])<0.000000600 and abs(self.goal_point[1]-self.curr_point[1])<0.000000600:
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
                        self.change(3)
                else:
                    self.path_plan()
        elif self.drone_state==3:##land
            if abs(self.error[2])<0.01:
                self.change(4)   
        elif self.drone_state==4: ##load unload
            if not self.isLoaded:    
                self.check_pick_gripper()                
                self.grip_check = True ##change the goal point in gripper
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                while self.grip_check == True:
                    if self.gripper == "True":
                        if self.ret == False:
                            self.set_point[0],self.set_point[1],self.set_point[2]=self.curr_point[0],self.curr_point[1],max(self.set_point[2],self.alt_setpoint[n]+15)
                            # print(self.set_point[2])
                            self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[n],self.long_setpoint[n],self.alt_setpoint[n]+15
                            self.grip_check = False
                            break
                        else:
                            self.set_point[0],self.set_point[1],self.set_point[2]=self.curr_point[0],self.curr_point[1],max(self.set_point[2],self.Return[self.point[self.n]][2]+15)
                            # print(self.set_point[2])
                            self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.Return[self.point[self.n]][0],self.Return[self.point[self.n]][1],self.Return[self.point[self.n]][2]+15
                            self.grip_check = False
                            break                            
                    else:
                        continue                 
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
                self.drone_state=1
            else:
                rospy.signal_shutdown("reached")

    # function to check if gripper is true or not and request for gripping
    def check_pick_gripper(self):
        # print(self.gripper)
        while self.gripper==False:
            rospy.logwarn("gripping unavailable")
            continue
        res=GripperResponse()
        
        while not res.result:
            res = self.gripper_srv(True)
            rospy.loginfo("Pick up successful")
        self.isLoaded=True

    # function to generate setpoints for wall following according to sensor readings
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

        elif (1<self.obs_dist[self.regions["front"]]<5 or 1<self.obs_dist[self.regions["top"]]<5 or 1<self.obs_dist[self.regions["back"]]<5) and self.obs_dist[self.regions["right"]]>5  and self.obs_dist[self.regions["left"]]>5:
            self.check = True
            self.sensor = "F or B"
            if (self.dx > 0 and self.dy > 0) or (self.dx < 0 and self.dy < 0):
                if abs(self.dx) > abs(self.dy) :
                    self.set_point[0]=self.prev_setpoint[0]+self.dx
                    self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
                else :
                    self.set_point[0]=self.prev_setpoint[0]+self.dy*6
                    self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            elif (self.dx < 0 and self.dy > 0) or (self.dx > 0 and self.dy < 0):
                if abs(self.dx) > abs(self.dy) :
                    self.set_point[0]=self.prev_setpoint[0]+self.dx
                    self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
                else :
                    self.set_point[0]=self.prev_setpoint[0]-self.dy*6
                    self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25                         
            rospy.loginfo("   F  ")

        elif (1<self.obs_dist[self.regions["right"]]<5 or 1<self.obs_dist[self.regions["left"]]<5) and self.obs_dist[self.regions["front"]]>5 and self.obs_dist[self.regions["back"]]>5 :
            self.check = True
            self.sensor = "L or R"
            if (self.dx > 0 and self.dy > 0) or (self.dx < 0 and self.dy < 0):
                if abs(self.dx) > abs(self.dy) :
                    self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                    self.set_point[1]=self.prev_setpoint[1]+self.dx
                else:
                    self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                    self.set_point[1]=self.prev_setpoint[1]+self.dy*6   
            elif (self.dx < 0 and self.dy > 0) or (self.dx > 0 and self.dy < 0):
                if abs(self.dx) > abs(self.dy) :
                    self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                    self.set_point[1]=self.prev_setpoint[1]-self.dx
                else:
                    self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                    self.set_point[1]=self.prev_setpoint[1]+self.dy*6    
            rospy.loginfo("   R   ")

        elif self.obs_dist[self.regions["right"]]>6 and self.obs_dist[self.regions["front"]]>6 and self.obs_dist[self.regions["back"]]>6 and self.obs_dist[self.regions["left"]]>6:
            self.set_point[0] = self.prev_setpoint[0] + self.dx/50
            self.set_point[1] = self.prev_setpoint[1] + self.dy/50   

    # function to give offset distance after obstacle avoidance/ wall following
    def sensor_reading(self):
        if self.sensor == "F or B":
            print("Front or Back")
            if self.dx > 0 :
                self.set_point[0]=self.prev_setpoint[0] + 5/110692.0702932625
            elif self.dx < 0 :
                self.set_point[0]=self.prev_setpoint[0] - 5/110692.0702932625
        elif self.sensor == "L or R" :
            print("Left or Right")
            if self.dy > 0 :
                self.set_point[1] = self.prev_setpoint[1] + 5/105292.0089353767                        
            elif self.dy < 0 :
                self.set_point[1] = self.prev_setpoint[1] - 5/105292.0089353767                        

    # function to follow the path using the setpoints(Bug0 approach)
    def path_plan(self):
        if ((self.obs_dist[0]<7 and self.obs_dist[0]>1) or (self.obs_dist[1]<7 and self.obs_dist[1]>1) or (self.obs_dist[2]<7 and self.obs_dist[2]>1) or (self.obs_dist[3]<7 and self.obs_dist[3]>1)):
            self.wall_foll_waypoint_gen()
            self.prev_setpoint[0] = self.set_point[0]
            self.prev_setpoint[1] = self.set_point[1]             
        else:
            if self.check == True:
                self.sensor_reading()
                # if self.sensor == "F or B":
                #     if abs(self.set_point[0]-self.curr_point[0]) < 0.000010000 :
                #         print("Oye Oye")
                #         self.prev_setpoint[0] = self.set_point[0]
                #         self.prev_setpoint[1] = self.set_point[1]                     
                #         self.distance()
                #         print(self.dist)
                #         self.check = False
                # elif self.sensor == "L or R":
                if abs(self.set_point[1]-self.curr_point[1]) < 0.000005000 :
                    # print("Oye Oye")
                    self.prev_setpoint[0] = self.set_point[0]
                    self.prev_setpoint[1] = self.set_point[1]                     
                    self.distance()
                    # print(self.dist)
                    self.check = False

            else :
                self.set_point[0] = self.prev_setpoint[0] + self.dx
                self.set_point[1] = self.prev_setpoint[1] + self.dy

                self.prev_setpoint[0] = self.set_point[0]
                self.prev_setpoint[1] = self.set_point[1]        

# main driver code
if __name__ == "__main__":
    # creating objects for different setpoints
    e_drone = edrone()
    while not rospy.is_shutdown():
        e_drone.handler()
        e_drone.pid()
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        e_drone.r.sleep()
=======
        e_drone.r.sleep()
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        e_drone.r.sleep()
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
=======
        e_drone.r.sleep()
>>>>>>> 53f69b2110d5184ae3b4685cebf66d78f522ea5a
