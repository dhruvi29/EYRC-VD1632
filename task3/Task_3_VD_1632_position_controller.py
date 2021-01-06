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

        #initialising current, set and initial points
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

        # defining regions
        self.regions ={
        "right" : 0 ,
        "back" : 1 ,
        "left" : 2,
        "front" : 3
        }

        # required variables
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

        self.err_x_m = 0.0
        self.err_y_m = 0.0
        self.marker_id = 1
        self.detect = False
        self.lat_setpoint=[0,0,0]
        self.long_setpoint=[0,0,0]
        self.alt_setpoint=[0,0,0]
        self.loc_count=-1
        self.isDetected=False
        self.stopDetection=False
        #--------------------------------------------------------------------------------------------------------------
        # Publishing on Topics 
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.marker_ID_pub = rospy.Publisher('/marker_ID', Int32, queue_size=1)

        # The below can be uncommented while tuning latitude, longitude and altitude->
        # self.latitude_error_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        # self.longitude_error_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        # self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)

        # Subscriptions on topics /edrone/gps, /pid_tuning_pitch,. /pid_tuning_roll, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid) 
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid) 
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        # rospy.Subscriber('/edrone/gripper_check',String,self.gripper_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        # rospy.Subscriber('/qrscan', qr_code, self.qr_callback)
        rospy.Subscriber('/edrone/marker_data',MarkerData,self.marker_callback)
        rospy.Subscriber("/isDetected",Bool,self.detection_clbk)
        # rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

        # self.gripper_srv=rospy.ServiceProxy('/edrone/activate_gripper',Gripper)

    #------------------------------------------------------------------------------------------------------------------
    # Callback functions
    
    def marker_callback(self, msg):
        self.marker_id = msg.marker_id
        self.err_x_m = msg.err_x_m
        self.err_y_m = msg.err_y_m

    # def qr_callback(self,msg):
    #     #print('in qr callback')
    #     self.qrcode[0]= msg.latitude
    #     self.qrcode[1]= msg.longitude
    #     self.qrcode[2]= msg.altitude
    #     rospy.loginfo('QR: %f %f %f',self.qrcode[0],self.qrcode[1],self.qrcode[2])

    def range_finder_top_callback(self, msg):
        self.obs_dist[self.regions["right"]] = msg.ranges[0]
        self.obs_dist[self.regions["back"]] = msg.ranges[1]
        self.obs_dist[self.regions["left"]] = msg.ranges[2]
        self.obs_dist[self.regions["front"]] = msg.ranges[3]
        
  
    # def gripper_callback(self,msg):
    #     self.gripper=msg.data

    def gps_callback(self, msg):
        self.curr_point[0] = msg.latitude
        self.curr_point[1] = msg.longitude
        self.curr_point[2] = msg.altitude
      
    # the below can be uncommented while tuning latitude, longitude and altitude 
    # def altitude_range_callback(self, msg):
        # self.altitude_range_curr = msg.ranges[0]

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

    # function to calculate setpoints
    def distance(self):
        self.dx = (self.goal_point[0] - self.set_point[0])/250
        self.dy = (self.goal_point[1] - self.set_point[1])/250

    # handler function -> driver code
    def handler(self):
        if self.drone_state==0:#change to takeoff -> state=1
            self.set_point[0]=self.initial_point[0]
            self.set_point[1]=self.initial_point[1]
            self.set_point[2]=self.initial_point[2]
            self.set_point[2]+=self.fly_hieght
            self.drone_state=1
        elif self.drone_state==1:
            rospy.loginfo("taking off")
            if abs(self.error[2])<0.1:
                self.prev_setpoint[0],self.prev_setpoint[1],self.prev_setpoint[2] = self.set_point[0],self.set_point[1],self.set_point[2],
                self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.loc_count+1],self.long_setpoint[self.loc_count+1],self.alt_setpoint[self.loc_count+1]
                self.drone_state=2
        elif self.drone_state==2:
            self.distance()
            if self.loc_count==-1:
                self.loc_count+=1
                self.set_point[2]=max(self.set_point[2],self.alt_setpoint[self.loc_count]+self.fly_hieght)
                self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.loc_count],self.long_setpoint[self.loc_count],self.alt_setpoint[self.loc_count]
            elif self.loc_count==0:
                rospy.loginfo("point 1")
                if self.detect:
                    if not self.isDetected:
                        self.set_point[2]+=0.1
                        self.goal_point[2]+=0.1
                        print(self.set_point[2])

                    else:
                        self.stopDetection=True
                        self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
                        self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767
                        if -1 < self.err_x_m < 1 and -1 < self.err_y_m < 1 :
                            self.set_point[2] = self.alt_setpoint[self.loc_count]+5                        
                        if -0.2 < self.err_x_m < 0.2 and -0.2 < self.err_y_m < 0.2 :
                            self.loc_count=1
                            self.set_point[2]=max(self.set_point[2],self.alt_setpoint[self.loc_count]+self.fly_hieght)
                            self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.loc_count],self.long_setpoint[self.loc_count],self.alt_setpoint[self.loc_count]   
                            self.detect = False
                            self.stopDetection=False
                elif abs(self.goal_point[0]-self.curr_point[0])<0.00000500 and abs(self.goal_point[1]-self.curr_point[1])<0.00000500:
                    self.set_point[2]=self.goal_point[2]+1
                    if(abs(self.set_point[2]-self.curr_point[2])<0.1 ):
                        self.detect = True
                    
            elif self.loc_count==1:
                rospy.loginfo("point 2")
                if self.detect == True :
                    if not self.isDetected:
                        self.set_point[2]+=0.1
                        self.goal_point[2]+=0.1
                    else:
                        self.stopDetection=True
                        self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
                        self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767
                        if -1 < self.err_x_m < 1 and -1 < self.err_y_m < 1 :
                            self.set_point[2] = self.alt_setpoint[self.loc_count]+5
                        if -0.2 < self.err_x_m < 0.2 and -0.2 < self.err_y_m < 0.2 :
                            self.loc_count=2
                            self.set_point[2]=max(self.set_point[2],self.alt_setpoint[self.loc_count]+self.fly_hieght)
                            self.goal_point[0],self.goal_point[1],self.goal_point[2]=self.lat_setpoint[self.loc_count],self.long_setpoint[self.loc_count],self.alt_setpoint[self.loc_count]   
                            self.detect = False
                            self.stopDetection=False
                elif abs(self.goal_point[0]-self.curr_point[0])<0.000050 and abs(self.goal_point[1]-self.curr_point[1])<0.0000050:
                    self.set_point[2]=self.goal_point[2]+1
                    if(abs(self.set_point[2]-self.curr_point[2])<0.1 ):
                        self.detect = True


            elif self.loc_count==2:
                rospy.loginfo("point 3")
                if self.detect == True :
                    if not self.isDetected:
                        self.set_point[2]+=0.1
                        print(self.set_point[2])
                    else:
                        self.stopDetection=True
                        self.goal_point[0] = self.curr_point[0]-self.err_x_m/110692.0702932625
                        self.goal_point[1] = self.curr_point[1]-self.err_y_m/105292.0089353767
                        if -0.2 < self.err_x_m < 0.2 and -0.2 < self.err_y_m < 0.2 :
                            self.drone_state=3
                            self.goal_point[2],self.set_point[2]=self.alt_setpoint[2],self.alt_setpoint[2]
                            self.detect = False
                            self.stopDetection=False
                elif abs(self.goal_point[0]-self.curr_point[0])<0.0000050 and abs(self.goal_point[1]-self.curr_point[1])<0.0000050:
                    self.set_point[2]=self.goal_point[2]+1
                    if(abs(self.error[2])<0.2 ):
                        self.detect = True

            else:
                rospy.signal_shutdown("Bye")
            self.marker_ID_pub.publish(self.loc_count+1)


    def check_pick_gripper(self):
        print(self.gripper)
        while self.gripper==False:
            rospy.logwarn("gripping unavailable")
            continue
        res=GripperResponse()
        
        while not res.result:
            res = self.gripper_srv(True)
            rospy.loginfo("Pick up successful")
        self.isLoaded=True

    # function to generate waypoints in path planning
    def wall_foll_waypoint_gen(self):
        if 1<self.obs_dist[self.regions["right"]]<7 and 1<self.obs_dist[self.regions["left"]]<7 and self.obs_dist[self.regions["front"]]>10 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]
        elif 1<self.obs_dist[self.regions["right"]]<7 and 1<self.obs_dist[self.regions["front"]]<10 and self.obs_dist[self.regions["left"]]>7 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]-self.dy/1.25
            # rospy.loginfo("R   F")

        elif 1<self.obs_dist[self.regions["left"]]<7 and 1<self.obs_dist[self.regions["front"]]<10 and self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]+self.dy/1.25
            # rospy.loginfo("L   F")

        elif 1<self.obs_dist[self.regions["left"]]<7 and self.obs_dist[self.regions["front"]]>10 and self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]+self.dx*1.25
            self.set_point[1]=self.prev_setpoint[1]#+self.dy/1.25
            # rospy.loginfo("   L   ")
        elif 1<self.obs_dist[self.regions["right"]]<7 and self.obs_dist[self.regions["front"]]>10 and self.obs_dist[self.regions["left"]]>7 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]+self.dx*1.25
            self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            # rospy.loginfo("   R  ")

        elif 1<self.obs_dist[self.regions["front"]]<10 and self.obs_dist[self.regions["right"]]>7 and self.obs_dist[self.regions["left"]]>7 and self.obs_dist[self.regions["back"]]>10:
            self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]+self.dy*3
            # rospy.loginfo("   F***")

    # path planning function
    def path_plan(self):
        if ((self.obs_dist[0]<7 and self.obs_dist[0]>1)or (self.obs_dist[2]<7 and self.obs_dist[2]>1) or (self.obs_dist[3]<10 and self.obs_dist[3]>1)):
            self.wall_foll_waypoint_gen()
        else:
            self.set_point[0] = self.prev_setpoint[0] + self.dx
            self.set_point[1] = self.prev_setpoint[1] + self.dy

        self.prev_setpoint[0] = self.set_point[0]
        self.prev_setpoint[1] = self.set_point[1] 


# main driver code
if __name__ == "__main__":
    # creating object for drone
    e_drone = edrone()
    # building setpoints
    e_drone.lat_setpoint=[18.9993675932,18.9990965928,18.9990965925]
    e_drone.long_setpoint=[72.0000569892,72.0000664814,71.9999050292 ]
    e_drone.alt_setpoint=[10.7,10.75,22.2]
    
    while not rospy.is_shutdown():
        e_drone.handler()
        e_drone.path_plan()
        e_drone.pid()
        e_drone.r.sleep()
