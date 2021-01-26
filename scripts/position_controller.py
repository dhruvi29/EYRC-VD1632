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

regions ={
        "right" : 0 ,
        "back" : 1 ,
        "left" : 2,
        "front" : 3
        }

#Creating Class For implementing given task
class edrone():
    # constructor
    #default object constructor-> gets called everytime when an instance is initialzed
    def __init__(self):
        print("init called")
        #initiasing ros node with name position_controller
        rospy.init_node('position_controller')

        # initialising set points as passed from parameters
        self.goal_point = [19.000000001,71.999957578,8.44100417359]
        #initialising current points
        self.curr_point = [19.0,72.0, 8.440994388]
        self.set_point = [19.0,72.0, 8.440994388]
        self.set_point[2]+=3.00
        self.B2 = [19.0007030405,71.9999429002,22.1600026799]

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

        self.sample_time = 0.06

        # state 1)takeoff 2)fly 3)land 4)gripper check,pick and drop,QR
        self.drone_state=1
        self.isLoaded=False
        self.fly_hieght=3.0000
        self.gripper=False
        self.dist = 0.0
        self.t = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.prev_setpoint = [0.0,0.0]
        self.r=rospy.Rate(1/self.sample_time)

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
        self.obs_dist[regions["right"]] = msg.ranges[0]
        self.obs_dist[regions["back"]] = msg.ranges[1]
        self.obs_dist[regions["left"]] = msg.ranges[2]
        self.obs_dist[regions["front"]] = msg.ranges[3]
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
        # self.dist = math.sqrt(math.pow(110692.0702932625 * (goal_longitude - initial_longitude) , 2) + math.pow(105292.0089353767 * (goal_latitude - initial_latitude) , 2))
        # self.t = self.dist*14
        self.dx = (goal_latitude - initial_latitude)/1000#self.t
        self.dy = (goal_longitude - initial_longitude)/1000#self.t
        # print(self.dx,self.dy)

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
            # print(self.set_point[2])
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
                if abs(self.goal_point[0]-self.curr_point[0])<0.000001000 and abs(self.goal_point[1]-self.curr_point[1])<0.000001000:
			        self.gripper_srv(False)
			        self.change(3)
            else:
                self.path_plan()
        elif self.drone_state==3:##land
            # rospy.loginfo("curr point lat: %f,long: %f altitude: %f",self.curr_point[0],self.curr_point[1],self.curr_point[2])
            if abs(self.error[2])<0.01:
                self.change(4)   
        elif self.drone_state==4: ##load unload
            if not self.isLoaded:
                self.check_pick_gripper() ##change the goal point in gripper
                # while(self.qrcode[0]==0):
                #     rospy.logwarn("waiting for QR")
                #     continue
                self.goal_point= self.B2 
                # self.QR()
                # print(self.goal_point)
                # self.goal_point=[19.00000,72.0000,8.44]
                r.sleep()
                self.change(1)
            else:
                # self.drop_parcel()
                rospy.signal_shutdown("reached")

    def check_pick_gripper(self):
        print(self.gripper)
	
        while self.gripper=="False":
            rospy.logwarn("gripping unavailable")
            continue
        res=GripperResponse()
        print(res)
       	while not res.result:
            res = self.gripper_srv(True)
            rospy.loginfo("Pick up successful")
        self.isLoaded=True

    def wall_foll_waypoint_gen(self):
        global regions
        if 1<self.obs_dist[regions["right"]]<6 and 1<self.obs_dist[regions["left"]]<6 and self.obs_dist[regions["front"]]>6 and self.obs_dist[regions["back"]]>6:
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]
        elif 1<self.obs_dist[regions["right"]]<6 and 1<self.obs_dist[regions["front"]]<6 and self.obs_dist[regions["left"]]>6 and self.obs_dist[regions["back"]]>6:
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]-self.dy/1.25
            rospy.loginfo("R   F")
        elif 1<self.obs_dist[regions["left"]]<6 and 1<self.obs_dist[regions["front"]]<6 and self.obs_dist[regions["right"]]>6 and self.obs_dist[regions["back"]]>6:
            self.set_point[0]=self.prev_setpoint[0]-self.dx/6
            self.set_point[1]=self.prev_setpoint[1]+self.dy/1.25
            rospy.loginfo("L   F")

        elif 1<self.obs_dist[regions["left"]]<6 and self.obs_dist[regions["front"]]>6 and self.obs_dist[regions["right"]]>6 and self.obs_dist[regions["back"]]>6:
            if self.dx > self.dy :
                self.set_point[0]=self.prev_setpoint[0]+self.dx*6
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            else :
                self.set_point[0]=self.prev_setpoint[0]+self.dy*6
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            rospy.loginfo("   L   ")
        elif 1<self.obs_dist[regions["right"]]<6 and self.obs_dist[regions["front"]]>6 and self.obs_dist[regions["left"]]>6 and self.obs_dist[regions["back"]]>6:
            if self.dx > self.dy :
                self.set_point[0]=self.prev_setpoint[0]+self.dx*6
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            else :
                self.set_point[0]=self.prev_setpoint[0]+self.dy*6
                self.set_point[1]=self.prev_setpoint[1]#-self.dy/1.25
            rospy.loginfo("   R  ")

        elif 1<self.obs_dist[regions["front"]]<6 and self.obs_dist[regions["right"]]>6 and self.obs_dist[regions["left"]]>6 and self.obs_dist[regions["back"]]>6:
            if self.dx > self.dy :
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]+self.dx*6
            else :
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]+self.dy*6
            rospy.loginfo("   F***")
        elif self.obs_dist[regions["front"]]>6 and self.obs_dist[regions["right"]]>6 and self.obs_dist[regions["left"]]>6 and 1<self.obs_dist[regions["back"]]<6:
            if self.dx > self.dy :
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]+self.dx*6
            else:
                self.set_point[0]=self.prev_setpoint[0]#-self.dx/6
                self.set_point[1]=self.prev_setpoint[1]+self.dy*6                
            rospy.loginfo("   F-***")


        

    
    def path_plan(self):
        if ((self.obs_dist[0]<6 and self.obs_dist[0]>1)or (self.obs_dist[1]<6 and self.obs_dist[1]>1)or (self.obs_dist[2]<6 and self.obs_dist[2]>1) or (self.obs_dist[3]<6 and self.obs_dist[3]>1)):
            self.wall_foll_waypoint_gen()
            # rospy.logerr("wall following")
        else:
            self.set_point[0] = self.prev_setpoint[0] + self.dx
            self.set_point[1] = self.prev_setpoint[1] + self.dy

        self.prev_setpoint[0] = self.set_point[0]
        self.prev_setpoint[1] = self.set_point[1] 
            # rospy.logwarn("no obstacle")

        # print(self.set_point[0],self.set_point[1])


        
        

# main driver code
if __name__ == "__main__":
    # print("init main")
    # creating objects for different setpoints
    e_drone = edrone()
    r = rospy.Rate(1/e_drone.sample_time)  
    e_drone.change(1)
    while not rospy.is_shutdown():
        e_drone.handler()
        e_drone.pid()
        r.sleep()
