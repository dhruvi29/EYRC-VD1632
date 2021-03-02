#!/usr/bin/env python2

# importing required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix

# global flag variables
flag =[0,0,0]
i1=0
i2=0
i3=0

#Creating Class For implementing given task
class edrone():
    # constructor
    #default object constructor-> gets called everytime when an instance is initialzed
    def __init__(self,set_point):
        #initiasing ros node with name position_controller
        rospy.init_node('position_controller')

        # initialising set points as passed from parameters
        self.set_point = set_point

        #initialising current points
        self.curr_point = [0.0,0.0,0.0]

        #initialsing Roll, Pitch and Yaw 
        self.attitude_cmd = edrone_cmd()
        self.attitude_cmd.rcRoll = 1500.0
        self.attitude_cmd.rcPitch = 1500.0
        self.attitude_cmd.rcYaw = 1500.0
        self.attitude_cmd.rcThrottle = 1500.0
        self.base_value = 1500
        
        # setting PID constants based on tuning
        # self.Kp = [1500*100,1500*100, 800*0.06]
        # self.Kd = [1486*1000,1486*1000, 1550.0*0.3]
        # self.Ki = [0, 0, 0*0.008]
        self.Kp = [0.06*5100*176, 1243* 0.06*5100, 1500*0.06]
        self.Ki = [0.0, 0.0, 0.0*0.008]
        self.Kd = [0.3*20500*873, 2102*0.3*20500, 5500*0.3]

        # calculating errors
        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.iterm = [0.0, 0.0, 0.0]
        self.pErr = [0.0, 0.0, 0.0]
        self.output = [0.0, 0.0, 0.0]
        self.dErr = [0.0, 0.0, 0.0]

        self.sample_time = 0.06

        #--------------------------------------------------------------------------------------------------------------
        # Publishing on Topics - /drone_command , /latitude_error, /longitude_error, /altitude_error
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # The below can be uncommented while tuning latitude, longitude and altitude->
        # self.latitude_error_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        # self.longitude_error_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        # self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)

        # Subscriptions on topics /edrone/gps, /pid_tuning_pitch,. /pid_tuning_roll, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid) 
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid) 
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)

    #------------------------------------------------------------------------------------------------------------------
    # Callback functions
    def gps_callback(self, msg):
        self.curr_point[0] = msg.latitude
        self.curr_point[1] = msg.longitude
        self.curr_point[2] = msg.altitude
      
    # the below can be uncommented while tuning latitude, longitude and altitude 
    # def altitude_range_callback(self, msg):
        # self.altitude_range_curr = msg.ranges[0]
        # rospy.loginfo("curr altitude : %f", self.altitude_range_curr)

    # def latitude_set_pid(self, msg):
    #     self.Kp[0] = msg.Kp * (100)
    #     self.Ki[0] = msg.Ki * 10
    #     self.Kd[0] = msg.Kd * 1000

    # def longitude_set_pid(self, msg):
    #     self.Kp[1] = msg.Kp * (100)
    #     self.Ki[1] = msg.Ki * 10
    #     self.Kd[1] = msg.Kd * 1000

    # def altitude_set_pid(self, msg):
    #     self.Kp[2] = msg.Kp * 0.06
    #     self.Ki[2] = msg.Ki * 0.008
    #     self.Kd[2] = msg.Kd * 0.3
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
        self.dErr[0] = (self.error[0] - self.prev_error[0]) / self.sample_time
        self.dErr[1] = (self.error[1] - self.prev_error[1]) / self.sample_time
        self.dErr[2] = (self.error[2] - self.prev_error[2]) / self.sample_time
    
        # Final PID Equations ->
        self.output[0] = self.pErr[1] + self.Ki[1] * self.iterm[1] * self.sample_time + self.Kd[1] * self.dErr[1] 
        self.output[1] = self.pErr[0] + self.Ki[0] * self.iterm[0] * self.sample_time + self.Kd[0] * self.dErr[0] 
        self.output[2] = self.pErr[2] + self.Ki[2] * self.iterm[2] * self.sample_time + self.Kd[2] * self.dErr[2] 

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
        # self.latitude_error_pub.publish(self.error[0])
        # self.longitude_error_pub.publish(self.error[1])
        # self.altitude_error_pub.publish(self.error[2])


# main driver code
if __name__ == '__main__':
    flag
    # creating objects for different setpoints
    e_drone_1 = edrone([19.0000000000,72.0000000000,3.000000000])
    e_drone_2 = edrone([19.0000451704,72.0000000000,3.000000000])
    e_drone_3 = edrone([19.0000451704,72.0000000000, 0.31000000])
    r = rospy.Rate(1 / e_drone_1.sample_time)  
    #running the PID corresponding to the given setpoints and the limiting error conditions when rospy is sctive->
    while not rospy.is_shutdown():
        if flag[0]==0:
            e_drone_1.pid()
            if abs(e_drone_1.error[2])<0.08 :
                i1+=1
                if i1==10:
                    flag[0]=1

        elif flag[1]==0 and flag[0]==1 :
            e_drone_2.pid()
            if abs(e_drone_2.error[0])<=0.000001517:
                i2+=1
                if i2==10:
                    flag[1]=1
                    e_drone_3.error=e_drone_2.error
                    e_drone_3.prev_error=e_drone_2.prev_error
                    e_drone_3.iterm=e_drone_2.iterm
                    e_drone_3.pErr=e_drone_2.pErr
                
        elif flag[2]==0 and flag[0]==1 and flag[1]==1: 
            e_drone_3.pid()
            if abs(e_drone_3.error[2])<=0.001:
                e_drone_3.attitude_cmd.rcThrottle=1000
                e_drone_3.attitude_cmd.rcRoll=1000
                e_drone_3.attitude_cmd.rcPitch=1000
                e_drone_3.attitude_cmd.rcYaw=1000
                e_drone_3.attitude_cmd.drone_cmd_pub.publish(e_drone_3.attitude_cmd)   
                rospy.signal_shutdown("stop")   
        r.sleep()
        
    

