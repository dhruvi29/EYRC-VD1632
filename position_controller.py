#!/usr/bin/env python
#imporitng requires libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix

class edrone():
	#constructor
	def __init__(self):
		rospy.init_node('position_controller')
		#latitude, longitude, altitude 
		self.set_point=[0.0,0.0,3.0]
		
		self.curr_point=[0.0,0.0,0.0]

		self.attitude_cmd=edrone_cmd()
		self.attitude_cmd.rcRoll=1500.0
		self.attitude_cmd.rcPitch=1500.0
		self.attitude_cmd.rcYaw=1500.0
		self.attitude_cmd.rcThrottle=1490.0
		
		
		#self.attitude_cmd.aux1=0.0
		#self.attitude_cmd.aux2=0.0
		#self.attitude_cmd.aux3=0.0
		
		self.altitude_range_curr = 0.0
		self.altitude_range_set = 3.0
		self.altitude_range_error = 0.0
		self.altitude_range_prev_error = 0.0
		self.altitude_range_iterm = 0.0
		self.altitude_range_pErr = 0.0
		self.altitude_range_output = 0.0
		self.altitude_range_dErr = 0.0
		self.altitude_range_Kp = 0
		self.altitude_range_Kd = 0
		self.altitude_range_Ki = 0
		
		
		self.Kp=[0.0,0.0,0.0]
		self.Kd=[0.0,0.0,0.0]
		self.Ki=[0.0,0.0,0.0]

		self.error=[0.0,0.0,0.0]
		self.prev_error=[0.0,0.0,0.0]
		self.iterm=[0.0,0.0,0.0]
		self.pErr=[0.0,0.0,0.0]
		self.output=[0.0,0.0,0.0]
		self.dErr=[0.0,0.0,0.0]

		self.sample_time = 0.5
		
		#check for Float64
		
		self.drone_cmd_pub=rospy.Publisher('/drone_command',edrone_cmd,queue_size=10)
		self.latitude_error_pub=rospy.Publisher('/latitude_error',Float32 ,queue_size=10)
		self.longitude_error_pub=rospy.Publisher('/longitude_error',Float32 ,queue_size=10)
		self.altitude_error_pub=rospy.Publisher('/altitude_error',Float32 ,queue_size=10)
		
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.altitude_range_callback)
		
		rospy.Subscriber('/pid_tuning_latitude',PidTune,self.latitude_set_pid)
		rospy.Subscriber('/pid_tuning_longitude',PidTune,self.longitude_set_pid)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)

	def gps_callback(self,msg):
		self.curr_point[0]=msg.latitude
		self.curr_point[1]=msg.longitude
		self.curr_point[2]=msg.altitude


	def altitude_range_callback(self,msg):
		self.altitude_range_curr=msg.ranges[0]
		rospy.loginfo("curr altitude : %f",self.altitude_range_curr)
		

	def latitude_set_pid(self,msg):
		self.Kp[0]=msg.Kp*0.06
		self.Ki[0]=msg.Ki*0.008
		self.Kd[0]=msg.Kd*0.3

	def longitude_set_pid(self,msg):
		self.Kp[1]=msg.Kp*0.06
		self.Ki[1]=msg.Ki*0.008
		self.Kd[1]=msg.Kd*0.3

	def altitude_set_pid(self,msg):
		self.Kp[2]=msg.Kp*0.06
		self.Ki[2]=msg.Ki*0.008
		self.Kd[2]=msg.Kd*0.3
		self.altitude_range_Kp=msg.Kp*0.06
		self.altitude_range_Ki=msg.Ki*0.008
		self.altitude_range_Kd=msg.Kd*0.3

    

	def pid(self):
		self.error[0]=self.set_point[0]-self.curr_point[0]
		self.error[1]=self.set_point[1]-self.curr_point[1]
		self.error[2]=self.set_point[2]-self.curr_point[2]
		self.altitude_range_error=self.altitude_range_set - self.altitude_range_curr
		print(self.error[2])
		
		#proportional error
		self.pErr[0]=self.Kp[0]*(self.error[0])
		self.pErr[1]=self.Kp[1]*(self.error[1])
		self.pErr[2]=self.Kp[2]*(self.error[2])
		self.altitude_range_pErr=self.altitude_range_Kp*self.altitude_range_error
		
		#iterm(integral term)
		self.iterm[0]=(self.iterm[0]+self.error[0])
		self.iterm[1]=(self.iterm[1]+self.error[1])
		self.iterm[2]=(self.iterm[2]+self.error[2])
		self.altitude_range_iterm=(self.altitude_range_iterm+self.altitude_range_error)
				
		#differential term
		self.dErr[0]=(self.error[0]-self.prev_error[0])/self.sample_time
		self.dErr[1]=(self.error[1]-self.prev_error[1])/self.sample_time
		self.dErr[2]=(self.error[2]-self.prev_error[2])/self.sample_time
		self.altitude_range_dErr=(self.altitude_range_error-self.altitude_range_prev_error)/self.sample_time
		
		self.output[0]=self.pErr[0] + self.Ki[0]*self.iterm[0]*self.sample_time + self.Kd[0]*self.dErr[0]
		self.output[1]=self.pErr[1] + self.Ki[1]*self.iterm[1]*self.sample_time + self.Kd[1]*self.dErr[1]
		self.output[2]=self.pErr[2] + self.Ki[2]*self.iterm[2]*self.sample_time + self.Kd[2]*self.dErr[2] + self.attitude_cmd.rcThrottle
		self.altitude_range_output=self.altitude_range_pErr +(self.altitude_range_Ki*self.altitude_range_iterm)*self.sample_time + self.altitude_range_Kd*self.altitude_range_dErr + self.attitude_cmd.rcThrottle
		
		#if self.output[0]>1024: self.output[0]=1024
		#if self.output[1]>1024: self.output[1]=1024
		if self.output[2]>1510: self.output[2]=1510
		
		#if self.output[0]<0: self.output[0]=0
		#if self.output[1]<0: self.output[1]=0
		if self.output[2]<1490: self.output[2]=1490
		
		#updating output throttle(thrust)
		self.attitude_cmd.rcThrottle = self.output[2]
		#updating error
		self.prev_error[0]=self.error[0]
		self.prev_error[1]=self.error[1]
		self.prev_error[2]=self.error[2]
		self.altitude_range_prev_error=self.altitude_range_error
		
		#printing logs
		#rospy.loginfo('throttle value :%f',self.attitude_cmd.rcThrottle)
		self.drone_cmd_pub.publish(self.attitude_cmd)
		self.latitude_error_pub.publish(self.error[0])
		self.longitude_error_pub.publish(self.error[1])
		self.altitude_error_pub.publish(self.error[2])
		
		
#exception handling
if __name__ == '__main__':

	e_drone = edrone()
	r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()

