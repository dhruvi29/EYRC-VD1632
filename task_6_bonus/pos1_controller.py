#!/usr/bin/env python2
'''
# Team ID:          VD#1632
# Theme:            Vitarana Drone(VD)
# Author List:      Karthik Swaminathan, Dhruvi Doshi, Ninad Jangle, Dhairya Shah
# Filename:         position_controller.py
# Functions:        pid, distance, change, box_dropping, path_plan, handler, check_pick_gripper
                    sensor_reading, wall_foll_waypoint_gen 
# Global variables: counter
'''

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
counter = 0

#Creating Class For implementing given task
class edrone():
    def __init__(self):
        '''
        Purpose:
        ---
        This function is the constructor for instance of edrone() class. It initialises all the variables 
        related to the instance and contains publisher and subscriber objects. Writing and reading the .csv files 
        takes place here

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        This is a contructor. It is implicitly called on creating an instance.
        '''
        print("init called")

        #initiasing ros node with name position_controller
        rospy.init_node('position_controller')

        # initialising set points as passed from parameters

        #initialising current points
        # self.curr_point: Stores co-ordinates of current point from gps topic
        self.curr_point = [18.9998887906,72.0002184403, 16.757981]
        # self.set_point: Stores co-ordinates of desired points in its path
        self.set_point = [18.9998887906,72.0002184403, 16.757981]
        # self.initial_point: Stores co-ordinates of initial spawn point
        self.initial_point=[18.9998887906,72.0002184403, 16.757981]

        #initialsing Roll, Pitch and Yaw 
        # self.attitude_cmd: Creates object having data members rcRoll, rcPitch, rcYaw and rcThrottle which is
        #                   attitude_controller node
        self.attitude_cmd = edrone_cmd()
        self.attitude_cmd.rcRoll = 1500.0
        self.attitude_cmd.rcPitch = 1500.0
        self.attitude_cmd.rcYaw = 1500.0
        self.attitude_cmd.rcThrottle = 1500.0
        # self.base_value: Base value for roll, pitch, yaw
        self.base_value = 1500

        # setting PID constants based on tuning
        # self.Kp: List of Proportional constants for latitiude, longitude and altitude PID respectively
        # self.Kp = [495*200, 470*200, 1500*0.06]
        self.Kp = [460*200, 481*200, 1500*0.06]

        # self.Ki: List of Integral constants for latitiude, longitude and altitude PID respectively
        self.Ki = [0.0, 0.0, 0.0*0.008]
        # self.Kd: List of Differential constants for latitiude, longitude and altitude PID respectively
        # self.Kd = [745*10000, 760*10000, 5550*0.3] #5600
        self.Kd = [733*10000, 720*10000, 5700*0.3] #5600


        # self.Kp = [460*200, 435*200, 1500*0.06]
        # self.Ki = [0.0, 0.0, 0.0*0.008]
        # self.Kd = [740*10000, 755*10000, 5550*0.3]        

        # calculating errors
        # self.error: List of errors in latitude, longitude, altitude between current point and given set point
        self.error = [999.00, 999.0, 999.00]
        # self.prev_error: List of errors in previous iteration latitude, longitude, altitude between current point and given set point
        self.prev_error = [0.0, 0.0, 0.0]
        # self.iterm: List of cummulative integral errors in latitude, longitude, altitude between current point and given set point
        self.iterm = [0.0, 0.0, 0.0]
        # self.pErr: List of proportional errosr in latitude, longitude, altitude between current point and given set point
        self.pErr = [0.0, 0.0, 0.0]
        # self.output: List of net output errors in latitude, longitude, altitude between current point and given set point
        self.output = [0.0, 0.0, 0.0]
        # self.dErr: List of differential errors in latitude, longitude, altitude between current point and given set point
        self.dErr = [0.0, 0.0, 0.0]
        # self.obs_dist: List of obstacle distances within environment around the edrone
        self.obs_dist = [0.0,0.0,0.0,0.0,0.0]
        # self.qrcode: List of qr co-ordinates in latitude, longitude, altitude between current point and given set point
        self.qrcode = [0.0,0.0,0.0]

        # self.sample_time: 1/frequency of delivery of messages to topics
        self.sample_time = 0.06
        # self.sensor: identifies the region of obstacle
        self.sensor = ""
        # self.regions: dictionary mapping regions to index numbers
        self.regions ={
        "front" : 0 ,
        "right" : 1 ,
        "back" : 2,
        "left" : 3,
        "top" : 4
        }

        # self.drone_state: determines motion state of drone
        self.drone_state=0
        # self.isLoaded: Boolean to represent the gripping of the parcel box with the edrone
        self.isLoaded=False
        # self.fly_height: Offset flying height for the drone
        self.fly_height=4.0000
        # self.gripper: Boolean to represent the gripper status of the edrone gripper service
        self.gripper=False
        # self.dist: Temporary variable to store distance
        self.dist = 0.0
        # self,no_points: Number of points for path 
        self.no_points = 0.0
        # self.dx: Incremental distance in latitude to cover the path no_point times
        self.dx = 0.0
        # self.dy: Incremental distance in longitude to cover the path no_point times
        self.dy = 0.0
        # self.prev_setpoint: List to store the last setpoint
        self.prev_setpoint = [18.9998887906,72.0002184403, 16.757981]
        # self.r: Publishing Frequency rate for messages
        self.r=rospy.Rate(1/self.sample_time)
        # self.grip_check: Boolean counter to check gripper status
        self.grip_check = False
        # self.detect: Boolean counter to indicate detection of marker
        self.detect = False
        # self.ret: Boolean counter to determine whether Return/Delivery process should take place
        self.ret = False
        # self.alt: variable to store data of range_finder_bottom
        self.alt = 0.0
        # self.lat_setpoint: List to store latitude setpoints for Delivery
        self.lat_setpoint=[]
        # self.long_setpoint: List to store longitude setpoints for Delivery
        self.long_setpoint=[]
        # self.alt_setpoint: List to store altitude setpoints for Delivery
        self.alt_setpoint=[]
        # self.ret_lat_setpoint: List to store latitude setpoints for Return
        self.ret_lat_setpoint=[]
        # self.ret_long_setpoint: List to store longitude setpoints for Return
        self.ret_long_setpoint=[]
        # self.ret_alt_setpoint: List to store altitude setpoints for Return
        self.ret_alt_setpoint=[]

        # self.Delivery: Dictionary to store Delivery grid co-ordinates 
        self.Delivery= {'A1': [18.9998102845 ,72.000142461,16.757981],'A2': [18.9998102845 ,72.000156707,16.757981],'A3': [18.9998102845 ,72.000170953,16.757981], 'B1': [18.999823836,72.000142461,16.757981],'B2': [18.999823836,72.000156707,16.757981],'B3': [18.999823836,72.000170953,16.757981],'C1': [18.999837387,72.000142461,16.757981], 'C2': [18.999837387,72.000156707, 16.757981],'C3': [18.999837387,72.000170953,16.757981]}
        # self.Return: Dictionary to store Return grid co-ordinates 
        self.Return= {'X1': [18.9999367615,72.000142461,16.757981],'X2': [18.9999367615,72.000156707,16.757981],'X3': [18.9999367615,72.000170953,16.757981], 'Y1': [18.999950313,72.000142461,16.757981],'Y2': [18.999950313,72.000156707,16.757981],'Y3': [18.999950313,72.000170953,16.757981],'Z1': [18.999963864,72.000142461,16.757981], 'Z2': [18.999963864,72.000156707,16.757981],'Z3': [18.999963864,72.000170953,16.757981]}
        # self.loc_count: Counter to store index of respective Delivery
        self.loc_count=0
        # self.ret_loc_count: Counter to store index of respective Return
        self.ret_loc_count=0
        # self.isDetected: Boolean to counter multiple detections
        self.isDetected=False
        # self.stopDetection: Boolean counter to indicate to stop detection 
        self.stopDetection=False
        # self.check: Boolean counter to give sensor readings on detecting an obstacle
        self.check=False

        # Getting delivery and return coordinates from csv file and storing it in lists

        # self.seq_delivery: Dictionary that stores delivery locations corresponding to delivery box names
        self.seq_delivery = {}
        # self.seq_return: Dictionary that stores return box locations corresponding to delivery box names
        self.seq_return = {}
        # self.seq_delivery_dist: Dictionary that stores distances between delivery grid and respective building location terrace  
        self.seq_delivery_dist = {}
        # self.seq_return_dist: Dictionary that stores distances between nearest return box and respective building location terrace 
        self.seq_return_dist = {}
        # self.sorted_delivery_index: List storing the FINAL sequenced indexes of the scheduled Delivery
        self.sorted_delivery_index = []
        # self.sorted_return_index: List storing the FINAL sequenced indexes of teh scheduled Return
        self.sorted_return_index = []

        # Reading manifest.csv file for fetching Delivery and Return co-ordinates
        with open('/home/dhairya/catkin_ws/src/vitarana_drone/scripts/bonus.csv', 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                if row[0]=="DELIVERY":
                    self.seq_delivery[row[1]] = [float(r) for r in row[2].split(";")]

                elif row[0]=="RETURN":    
                    self.seq_return[row[2]] = [float(r) for r in row[1].split(";")]

        # Sequencing deliveries as per distance between grid and approximate building point(largest first)
        for (key, val) in self.seq_delivery.items():
            m = math.sqrt(math.pow(110692.0702932625 * (val[0] - self.Delivery[key][0]) , 2) + math.pow(105292.0089353767 * (val[1] - self.Delivery[key][1]) , 2))
            self.seq_delivery_dist[key] = m
        sorted_delivery = sorted(self.seq_delivery_dist.values(),reverse=True)
        for n1 in sorted_delivery:
            for keys in self.seq_delivery_dist.keys():
                if  self.seq_delivery_dist[keys] == n1:
                    self.sorted_delivery_index.append(keys)
                    break 
            del self.seq_delivery_dist[keys]    

        # Sequencing returns as per distance of return box wrt each consecutive scheduled delivery(closest first)
        m = []
        self.seq_return2 = self.seq_return.copy()
        # finding nearest box from the terrace of a building
        for key in self.sorted_delivery_index:
            for key2, val2 in self.seq_return2.items():  
                m.append(math.sqrt(math.pow(110692.0702932625 * (self.seq_delivery[key][0] - val2[0]) , 2) + math.pow(105292.0089353767 * (self.seq_delivery[key][1] - val2[1]) , 2)))
            k = m.index(min(m))
            self.seq_return_dist[self.seq_return2.keys()[k]] = min(m)
            self.sorted_return_index.append(self.seq_return2.keys()[k])
            self.seq_return2.pop(self.seq_return2.keys()[k])            
            print(self.seq_return2)
            m = []
   

        # Writing the scheduled DELIVERY-RETURN order in sequenced_manifest.csv
        with open('/home/dhairya/catkin_ws/src/vitarana_drone/scripts/sequenced_manifest_bonus.csv', 'w') as file:
            writer = csv.writer(file)
            for c in range(len(self.sorted_delivery_index)):
                s = ";".join([str(self.seq_delivery[self.sorted_delivery_index[c]][0]),str(self.seq_delivery[self.sorted_delivery_index[c]][1]),str(self.seq_delivery[self.sorted_delivery_index[c]][2])])
                writer.writerow(["DELIVERY",self.sorted_delivery_index[c],s])
                print("DELIVERY: ",self.sorted_delivery_index[c])
                self.lat_setpoint.append(self.seq_delivery[self.sorted_delivery_index[c]][0])
                self.long_setpoint.append(self.seq_delivery[self.sorted_delivery_index[c]][1])
                self.alt_setpoint.append(self.seq_delivery[self.sorted_delivery_index[c]][2])

                s1 = ";".join([str(self.seq_return[self.sorted_return_index[c]][0]),str(self.seq_return[self.sorted_return_index[c]][1]),str(self.seq_return[self.sorted_return_index[c]][2])])
                writer.writerow(["RETURN",s1,self.sorted_return_index[c]])
                print("RETURN: ",self.sorted_return_index[c])
                self.ret_lat_setpoint.append(self.seq_return[self.sorted_return_index[c]][0])
                self.ret_long_setpoint.append(self.seq_return[self.sorted_return_index[c]][1])
                self.ret_alt_setpoint.append(self.seq_return[self.sorted_return_index[c]][2])


        #--------------------------------------------------------------------------------------------------------------
        # Publishing on Topics - /drone_command , /latitude_error, /longitude_error, /altitude_error
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        # The below can be uncommented while tuning latitude, longitude and altitude->
        # self.latitude_error_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        # self.longitude_error_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        # self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)

        # Publishing altitude to detect_marker_cascade node
        self.altitude_pub = rospy.Publisher('/altitude', Float32, queue_size=1)
        
        # The below can be uncommented while tuning latitude, longitude and altitude, and scanning qrcode ->
        # Subscriptions on topics /pid_tuning_pitch, /pid_tuning_roll, /pid_tuning_yaw
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid) 
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid) 
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        # rospy.Subscriber('/qrscan', qr_code, self.qr_callback)

        # Subscriptions on topics /edrone.gps, /edrone/gripper_check, /edrone/err_x_m, /edrone/err_y_m,
        #                         /isDetected, /edrone/range_finder_bottom
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/gripper_check',String,self.gripper_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber("/edrone/err_x_m", Float64, self.err_xm_callback)
        rospy.Subscriber("/edrone/err_y_m", Float64, self.err_ym_callback)
        rospy.Subscriber("/isDetected",Bool,self.detection_clbk)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

        # self.gripper_srv: Service object for Gripper Service
        self.gripper_srv=rospy.ServiceProxy('/edrone/activate_gripper',Gripper)

    #====================================================================================================================
    # Callback functions

    # Callback function for Reading QR co-ordinates
    # def qr_callback(self,msg):
    #     #print('in qr callback')
    #     self.qrcode[0]= msg.latitude
    #     self.qrcode[1]= msg.longitude
    #     self.qrcode[2]= msg.altitude
    #     rospy.loginfo('QR: %f %f %f',self.qrcode[0],self.qrcode[1],self.qrcode[2])

    # Callback function to updating error in x while detecting
    def err_xm_callback(self,msg):
        self.err_x_m = msg.data

    # Callback function to updating error in y while detecting
    def err_ym_callback(self,msg):
        self.err_y_m = msg.data

    # Callback function to update range_finder_top sensor values 
    def range_finder_top_callback(self, msg):
        self.obs_dist[self.regions["front"]] = msg.ranges[0]
        self.obs_dist[self.regions["right"]] = msg.ranges[1]
        self.obs_dist[self.regions["back"]] = msg.ranges[2]
        self.obs_dist[self.regions["left"]] = msg.ranges[3]
        self.obs_dist[self.regions["top"]] = msg.ranges[4]

    # Callback function to update gripper status from the service and availability
    def gripper_callback(self,msg):
        self.gripper=msg.data

    # Callback function to update gps co-ordinates
    def gps_callback(self, msg):
        self.curr_point[0] = msg.latitude
        self.curr_point[1] = msg.longitude
        self.curr_point[2] = msg.altitude
      
    # Callback function to update range_finder_top sensor values 
    def range_finder_bottom_callback(self, msg):
        self.alt = msg.ranges[0]

    # The below can be uncommented while tuning latitude, longitude and altitude, and scanning qrcode ->
    # Callback function to update latitude PID constants from sliders
    def latitude_set_pid(self, msg):
        self.Kp[0] = msg.Kp * (200)
        self.Ki[0] = msg.Ki * 10
        self.Kd[0] = msg.Kd * 10000

    # Callback function to update longitude PID constants from sliders
    def longitude_set_pid(self, msg):
        self.Kp[1] = msg.Kp * (200)
        self.Ki[1] = msg.Ki * 10
        self.Kd[1] = msg.Kd * 10000

    # Callback function to update altitude PID constants from sliders
    def altitude_set_pid(self, msg):
        self.Kp[2] = msg.Kp * 0.2
        self.Ki[2] = msg.Ki * 0.008
        self.Kd[2] = msg.Kd * 0.3

    # Callback function to update detection status for marker
    def detection_clbk(self,msg):
        if not self.stopDetection:
            self.isDetected=msg.data
            
    #================================================================================================================================
    def pid(self):
        '''
        Purpose:
        ---
        To reduce error in latitude, longitude and altitude as per given setpoints. 
        Main function for executing PID ALGORITHM

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        e_drone = edrone()
        e_drone.pid()
        '''

        # calculating errors for set_point with respect to current point
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
        
        # updating errors
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
   
        # publishing  on /edrone_command topic
        self.drone_cmd_pub.publish(self.attitude_cmd)

        # The below can be uncommented while tuning 
        # self.latitude_error_pub.publish(self.error[0])
        # self.longitude_error_pub.publish(self.error[1])
        # self.altitude_error_pub.publish(self.error[2])

    
    def distance(self):
        '''
        Purpose:
        ---
        function to calculate increment/decrement (dx, dy) in latitude and longitude as per given goal point and set point 
        based on the distance between them

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.distance()
        '''

        # Calculating distance between goal point and set point
        self.dist = math.sqrt(math.pow(110692.0702932625 * (self.goal_point[0] - self.set_point[0]) , 2) + math.pow(105292.0089353767 * (self.goal_point[1] - self.set_point[1]) , 2))
        # Calculating no of setpoints based on distance magnitude
        if 0<self.dist < 3 :
            self.no_points = self.dist*44 
        elif 3<self.dist < 6 :
            self.no_points = self.dist*26
        elif 6 <= self.dist < 10 :
            self.no_points = self.dist*16
        elif 10<= self.dist < 15:
            self.no_points = self.dist*4.25
        elif 15<= self.dist < 20:
            self.no_points = self.dist*4
        elif 20 <= self.dist < 40 :
            self.no_points = self.dist*3
        elif 40 <= self.dist < 60 :
            self.no_points = self.dist*2
        elif 60 <= self.dist < 80 :
            self.no_points = self.dist*1.65
        # elif 80 <= self.dist < 100 :
        #     self.no_points = self.dist*1.5
        # elif 100 <= self.dist < 120 :
        #     self.no_points = self.dist*1.3  
        # elif 120 <= self.dist < 140 :
        #     self.no_points = self.dist*1.2
        # elif 140<= self.dist < 160:
        #     self.no_points = self.dist*1*2
        # elif 160 <= self.dist < 200 :
        #     self.no_points = self.dist*1
        else :
            self.no_points = self.dist*1.1 #self.dist*0.81 1.1
        if self.no_points == 0:
            self.no_points = 1
        # Calculating incremental latitude and longitude distance          
        self.dx = (self.goal_point[0] - self.set_point[0])/self.no_points
        self.dy = (self.goal_point[1] - self.set_point[1])/self.no_points
        print(self.no_points)


    def change(self,n):
        '''
        Purpose:
        ---
        Function just to change states and provide goal point and set points.
        state 1=take off, state 2=fly state 3=land state 4=gripper check,pick drop 

        Input Arguments:
        ---
        `n` :  [Int32]
            State to which the drone state is to be changed.


        Returns:
        ---
        None

        Example call:
        ---
        self.change(2)
        '''
        # changing states
        self.drone_state=n
        rospy.loginfo("changing to drone_state %f ",self.drone_state)
        if self.drone_state ==1:
            self.set_point[0]=self.curr_point[0]
            self.set_point[1]=self.curr_point[1]
            self.set_point[2]=self.curr_point[2]
            if self.goal_point[2]>self.curr_point[2]:
                self.set_point[2]= self.fly_height+self.goal_point[2]
            else:
                self.set_point[2]+=self.fly_height
        elif self.drone_state==2:
            self.set_point[2]=self.goal_point[2]
            if self.goal_point[2]>self.curr_point[2]:
                self.set_point[2]+= self.fly_height
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
                    self.set_point[2]=float(self.ret_alt_setpoint[self.ret_loc_count])
                else:
                    self.set_point[2]=self.Return[self.sorted_return_index[self.ret_loc_count]][2]+1

    
    def box_dropping(self): 
        '''
        Purpose:
        ---
        Function to drop the parcel box on destination marker/return grid respectively.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.box_dropping()
        '''

        global counter           
        if self.ret == False:
            if self.detect:
                if not self.isDetected:
                    self.set_point[2]+=0.1
                    self.goal_point[2]+=0.1
                else:
                    self.stopDetection=True
                    print("detection success")
                    self.goal_point[0] = self.curr_point[0]-1*self.err_x_m/110692.0702932625  ##1
                    self.goal_point[1] = self.curr_point[1]-1*self.err_y_m/105292.0089353767
                    self.distance()
                    self.path_plan()
                    # if abs(self.goal_point[0]-self.curr_point[0])<0.00003000 and abs(self.goal_point[1]-self.curr_point[1])<0.00003000:
                    #     self.set_point[2] = self.alt_setpoint[n]+7                       
                    if -0.4 < self.err_x_m < 0.4 and -0.4 < self.err_y_m < 0.4 : ##0.4
                        self.set_point[2] = self.alt_setpoint[self.loc_count]+0.4  ##0.4
                        # self.set_point[2] = self.set_point[2]-0.1
                        if abs(self.set_point[2]-self.curr_point[2]) < 0.2:
                        # if abs(self.curr_point[2]-self.alt_setpoint[self.loc_count]) < 0.5:
                            self.gripper_srv(False)
                            self.gripper = False
                            self.loc_count+=1
                            self.set_point[0],self.set_point[1] = self.goal_point[0],self.goal_point[1]
                            
                            if self.loc_count > len(self.sorted_delivery_index): 
                                self.set_point[2]=max(self.set_point[2]+2,self.goal_point[2]+2)#self.fly_height)
                                self.goal_point=self.initial_point  
                            else:
                                self.set_point[0],self.set_point[1]= self.curr_point[0],self.curr_point[1]
                                # self.set_point[2] = self.alt_setpoint[self.loc_count-1]+2
                                self.goal_point[0],self.goal_point[1],self.goal_point[2] = float(self.ret_lat_setpoint[self.ret_loc_count]),float(self.ret_long_setpoint[self.ret_loc_count]),float(self.ret_alt_setpoint[self.ret_loc_count])+7
                                if self.set_point[2] > self.goal_point[2]:
                                    self.set_point[2]=max(self.set_point[2]+3,self.goal_point[2])#self.fly_height)
                                else:
                                    self.set_point[2]=max(self.set_point[2],self.goal_point[2])#self.fly_height)
                            self.detect = False
                            self.ret = True
                            self.stopDetection=False
                            while counter <100 :
                                counter+= 1
                            counter = 0    
                            self.drone_state=1
            elif abs(self.goal_point[0]-self.curr_point[0])<0.00002000 and abs(self.goal_point[1]-self.curr_point[1])<0.00002000:
                self.altitude_pub.publish(self.alt_setpoint[self.loc_count])
                if(abs(self.set_point[2]-self.curr_point[2])<0.3 ):
                    self.detect = True                   
            else:
                if abs(self.goal_point[0]-self.curr_point[0])<0.0003200 and abs(self.goal_point[1]-self.curr_point[1])<0.000320:
                    self.set_point[2]=self.alt_setpoint[self.loc_count]+9
                if abs(self.goal_point[0]-self.curr_point[0])<0.00018500 and abs(self.goal_point[1]-self.curr_point[1])<0.00018500:
                    self.set_point[0]=self.goal_point[0]
                    self.set_point[1]=self.goal_point[1]    
                else:                  
                    self.path_plan()
        else:
            if abs(self.goal_point[0]-self.curr_point[0])<0.0002400 and abs(self.goal_point[1]-self.curr_point[1])<0.0002400:
                self.set_point[2]=self.Return[self.sorted_return_index[self.ret_loc_count]][2]+10                   
            if abs(self.goal_point[0]-self.curr_point[0])<0.00017500 and abs(self.goal_point[1]-self.curr_point[1])<0.00017500:
                self.set_point[0]=self.goal_point[0]
                self.set_point[1]=self.goal_point[1]

                if abs(self.goal_point[0]-self.curr_point[0])<0.000002000 and abs(self.goal_point[1]-self.curr_point[1])<0.000002000:
                    self.set_point[2] = self.Return[self.sorted_return_index[self.ret_loc_count]][2]+0.3
                    if abs(self.set_point[2]-self.curr_point[2])<0.01:
                        self.ret = False
                        self.gripper_srv(False)                       
                        self.gripper = False
                        if self.ret_loc_count > len(self.sorted_return_index): 
                            self.goal_point=self.initial_point                      
                            self.set_point[2]=max(self.set_point[2],self.alt_setpoint[2]+self.fly_height)#self.fly_height)                            
                        else:
                            self.goal_point = self.Delivery[self.sorted_delivery_index[self.loc_count]]       
                            # self.goal_point=self.Delivery[self.sorted_delivery_index[self.n]] 
                            self.set_point[2]=max(self.set_point[2],self.goal_point[2])#self.fly_height)
                        while counter <100 :
                            counter+= 1
                        counter = 0 
                        self.ret_loc_count+=1
                        self.drone_state=1  
            else:
                self.path_plan()

    
    def handler(self):
        '''
        Purpose:
        ---
        Handler for carrying out different states of drone like takeoff, landing, calling funtions for box picking/dropping 

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        e_drone = edrone()
        e_drone.handler()
        '''

        global i
        # changing to takeoff -> state=1
        if self.drone_state==0:
            self.goal_point = self.Delivery[self.sorted_delivery_index[self.loc_count]]
            self.set_point[2]=self.curr_point[2]
            if self.goal_point[2]>self.curr_point[2]:
                self.set_point[2]= self.fly_height+self.goal_point[2]
            else:
                self.set_point[2]+=self.fly_height
            self.drone_state=1
        # taking off
        elif self.drone_state==1:
            self.isLoaded = False
            rospy.loginfo("taking off")
            if abs(self.error[2])<0.1:
                self.prev_setpoint[0],self.prev_setpoint[1],self.prev_setpoint[2] = self.set_point[0],self.set_point[1],self.set_point[2]         
                self.distance()                
                self.change(2)
       
        elif self.drone_state==2:
            if self.gripper == "True":
                self.box_dropping()
            else:
                if abs(self.goal_point[0]-self.curr_point[0])<0.00017500 and abs(self.goal_point[1]-self.curr_point[1])<0.00017500:
                    if self.ret == False:
                        self.set_point[2]=self.goal_point[2]+5   
                    else:                    
                        self.set_point[2]=float(self.ret_alt_setpoint[self.ret_loc_count])+5
                if abs(self.goal_point[0]-self.curr_point[0])<0.0001500 and abs(self.goal_point[1]-self.curr_point[1])<0.0001500:
                    self.set_point[0]=self.goal_point[0]
                    self.set_point[1]=self.goal_point[1]

                    if abs(self.goal_point[0]-self.curr_point[0])<0.000001500 and abs(self.goal_point[1]-self.curr_point[1])<0.000001500:
                        self.change(3)
                else:
                    self.path_plan()
        # land            
        elif self.drone_state==3: 
            if abs(self.error[2])<0.1:
                self.change(4)   
        # Load Unload
        elif self.drone_state==4: 
            if not self.isLoaded:    
                self.check_pick_gripper()                
                self.grip_check = True
                while self.grip_check == True:
                    if self.gripper == "True":
                        if self.ret == False:
                            self.set_point[0],self.set_point[1],self.set_point[2]=self.curr_point[0],self.curr_point[1],max(self.set_point[2],self.alt_setpoint[self.loc_count]+15)
                            # print(self.set_point[2])
                            self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.loc_count],self.long_setpoint[self.loc_count],self.alt_setpoint[self.loc_count]+15
                            self.grip_check = False
                            break
                        else:
                            self.set_point[0],self.set_point[1],self.set_point[2]=self.curr_point[0],self.curr_point[1],max(self.set_point[2],self.Return[self.sorted_return_index[self.ret_loc_count]][2]+15)
                            # print(self.set_point[2])
                            self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.Return[self.sorted_return_index[self.ret_loc_count]][0],self.Return[self.sorted_return_index[self.ret_loc_count]][1],self.Return[self.sorted_return_index[self.ret_loc_count]][2]+15
                            self.grip_check = False
                            break                            
                    else:
                        continue                 
                self.drone_state=1
            else:
                rospy.signal_shutdown("reached")


    def check_pick_gripper(self):
        '''
        Purpose:
        ---
        Function to check if gripper status is true or not and requests for gripping when available.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.check_pick_gripper()
        '''

        # print(self.gripper)
        while self.gripper==False:
            rospy.logwarn("gripping unavailable")
            continue
        
        # Fetching Gripper Response
        res=GripperResponse()
        
        # if grip is successsful
        while not res.result:
            res = self.gripper_srv(True)
            rospy.loginfo("Pick up successful")
        self.isLoaded=True
 

    def wall_foll_waypoint_gen(self):
        '''
        Purpose:
        ---
        Function to generate setpoints for wall following according to sensor readings and perform wall following.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.wall_foll_waypoint_gen()
        '''

        # giving setpoints as per sensor readings
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

    
    def sensor_reading(self):
        '''
        Purpose:
        ---
        Function to give offset distance after obstacle avoidance/ wall following.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self,sensor_reading()
        '''

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

    
    def path_plan(self):
        '''
        Purpose:
        ---
        Function to follow the planned path using the setpoints (using Bug0 Algorithm Approach).

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        e_drone = edrone()
        e_drone,path_plan()
        '''

        # Case for wall following
        if ((self.obs_dist[0]<7 and self.obs_dist[0]>1) or (self.obs_dist[1]<7 and self.obs_dist[1]>1) or (self.obs_dist[2]<7 and self.obs_dist[2]>1) or (self.obs_dist[3]<7 and self.obs_dist[3]>1)):
            self.wall_foll_waypoint_gen()
            self.prev_setpoint[0] = self.set_point[0]
            self.prev_setpoint[1] = self.set_point[1]             
        # Case for calculated path planning
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
                    self.prev_setpoint[0] = self.set_point[0]
                    self.prev_setpoint[1] = self.set_point[1]                     
                    self.distance()
                    # print(self.dist)
                    self.check = False

            else :
                self.distance() # ninu addition
                self.set_point[0] = self.prev_setpoint[0] + self.dx
                self.set_point[1] = self.prev_setpoint[1] + self.dy

                self.prev_setpoint[0] = self.set_point[0]
                self.prev_setpoint[1] = self.set_point[1]        


# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To create edrone instance and recursively call handler, pid functions until rospy node shutdowns.
if __name__ == "__main__":
    # creating objects for different setpoints
    e_drone = edrone()
    
    # running the handler, pid until the rospy node is shutdown
    while not rospy.is_shutdown():
        e_drone.handler()
        e_drone.pid()
        e_drone.r.sleep()
