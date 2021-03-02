#!/usr/bin/env python2

'''
# Team ID:          VD 1632
# Theme:            Vitarana Drone(VD)
# Author List:      Karthik Swaminathan, Dhruvi Doshi, Ninad Jangle, Dhairya Shah
# Filename:         attitude_position_controller.py
# Functions:        pid()
# Global variables: none
'''

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import rospy
import time
import tf
import math

#Creating Class For implementing given task
class Edrone():
    #default object constructor->gets called everytime when an instance is initialzed
    def __init__(self):
        '''
        Purpose:
        ---
        initialize class variables and set publishers and subscribers

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        This function is a constructor, gets implicity called on creation of instance of class Edrone
        e_drone=Edrone()
        '''

        # initializing ros node with name attitude_controller
        rospy.init_node('attitude_controller')  

        # self.drone_orientation_quaternion: This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # self.drone_orientation_euler: This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # self.setpoint_cmd: This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [1500.0, 1500.0, 1500.0]
        # self.Throttle: This is the throttle setpoint that will be received from the drone_command in the range from 1000 to 2000
        self.Throttle=0.0

        # self.setpoint_euler: The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        #----------------------------------------------------------------------------------------------------------------
        # self.pwm_cmd: Declaring pwm_cmd of message type prop_speed and initializing values
        #initialisng PWM speeds for motors
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        #-------------------------------------------------------------------------------------------------------------------
        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning the PID- r,p parameters 
        self.Kp = [2052*0.06 ,1266*0.06, 3362*0.6]
        self.Ki = [0, 4*0.008,192*0.08 ]
        self.Kd = [566*0.3, 240*0.3, 480*3]
        
        #----------------------------------------------------------------------------------------------------------
        # Initialising Variables for PID
        
        # self.prev_error: Stores errors of roll,pitch,yaw i.e (self.setpoint_euler-self.drone_orientation_euler)
        self.error = [0.0,0.0,0.0]
        # self.prev_error: Stores errors of previous iteration of roll,pitch,yaw
        self.prev_error = [0,0,0]
        # self.max_values: Stores maximum values of the propellers
        self.max_values = [1023, 1023, 1023, 1023]
        # self.min_values: Stores minimum values of the propellers
        self.min_values = [0, 0, 0, 0]
        # self.err_sum: cumulative sum of error over all iterations
        self.err_sum = [0.0,0.0,0.0]
        # differential error i.e. prev_error-error
        self.dErr = [0.0,0.0,0.0]
        # self.iterm_limit: a constant to limit the maximum value of err_sum
        self.iterm_limit = 1

        # self.sample_time: sample time for running PID
        self.sample_time = 0.06  # in seconds
        
        #------------------------------------------------------------------------------------------------------------
        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        
        # the below Publications can be uncommented during tuning
        self.roll_error_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.yaw_error_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)	  

        # -----------------------------------------------------------------------------------------------------------
        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
	    # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        # ------------------------------------------------------------------------------------------------------------


    #-------------------------------------------------------------------------------------------------------------------------
    #functions for callbacks to update values from subscriptions

    # Description: This callback function gets current roll,pitch,yaw status
    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    # Description: This callback recieves setpoints for roll,pitch,yaw and throttle from position_controller script
    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.Throttle = msg.rcThrottle
       

    # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    # def roll_set_pid(self, roll):
    #     self.Kp[0] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
    #     self.Ki[0] = roll.Ki * 0.008
    #     self.Kd[0] = roll.Kd * 0.3

    # # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
    # def pitch_set_pid(self, pitch):
    #     self.Kp[1] = pitch.Kp * 0.06  
    #     self.Ki[1] = pitch.Ki * 0.008
    #     self.Kd[1] = pitch.Kd * 0.3

    # def yaw_set_pid(self, yaw):
    #     self.Kp[2] = yaw.Kp * 0.6  
    #     self.Ki[2] = yaw.Ki * 0.08
    #     self.Kd[2] = yaw.Kd * 3


    # ----------------------------------------------------------------------------------------------------------------------
    def pid(self):
        '''
        Purpose:
        ---
        main function for implementing PID algorithm< Short-text describing the purpose of this function.
        Publishes propeller speed according to desired roll,pitch and yaw.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        e_drone = Edrone()
        e_drone.pid()
        '''

        # Converting quaternion to euler angles
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        #Mapping setpoints(1000 to 2000) to (-10 to 10 ) degrees         
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
        
        # Mapping the range of 1000 to 2000 to 0 to 1024 for throttle For 10 bit range PWM values
        self.Throttle = self.Throttle * 1.024 - 1024

        # Calculating the errors
        self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
        self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]
        self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]

        # Summing the errors for the integral term/cumulative error for PID
        self.err_sum[0] += self.error[0] * self.sample_time  
        self.err_sum[1] += self.error[1] * self.sample_time 
        self.err_sum[2] += self.error[2] * self.sample_time 

        #Limiting the error-sum term value to counter saturation
        if self.err_sum[0]>self.iterm_limit:self.err_sum[0]=1
        if self.err_sum[1]>self.iterm_limit:self.err_sum[1]=1
        if self.err_sum[2]>self.iterm_limit:self.err_sum[2]=1

        #calculating the differential error
        self.dErr[0] = (self.error[0] - self.prev_error[0]) / self.sample_time
        self.dErr[1] = (self.error[1] - self.prev_error[1]) / self.sample_time
        self.dErr[2] = (self.error[2] - self.prev_error[2]) / self.sample_time
	

        # PID Equations for ROLL, PITCH and YAW respectively
        self.out_roll = self.Kp[0] * self.error[0] + self.Ki[0] * self.err_sum[0] *self.sample_time+ self.Kd[0] * self.dErr[0]
        self.out_pitch = self.Kp[1] * self.error[1] + self.Ki[1] * self.err_sum[1] *self.sample_time+ self.Kd[1] * self.dErr[1]
        self.out_yaw = self.Kp[2] * self.error[2] + self.Ki[2] * self.err_sum[2] *self.sample_time+ self.Kd[2] * self.dErr[2]

        # Calculating the Propellor speeds for the Drone - MAIN PID PWM Equations
        self.pwm_cmd.prop1 = self.Throttle + self.out_roll - self.out_pitch - self.out_yaw
        self.pwm_cmd.prop2 = self.Throttle - self.out_roll - self.out_pitch + self.out_yaw
        self.pwm_cmd.prop3 = self.Throttle - self.out_roll + self.out_pitch - self.out_yaw
        self.pwm_cmd.prop4 = self.Throttle + self.out_roll + self.out_pitch + self.out_yaw

        # Limiting the propellor speeds within given Range
        if self.pwm_cmd.prop1 > self.max_values[0]:
            self.pwm_cmd.prop1 = self.max_values[0]
        if self.pwm_cmd.prop2 > self.max_values[1]:
            self.pwm_cmd.prop2 = self.max_values[1]
        if self.pwm_cmd.prop3 > self.max_values[2]:
            self.pwm_cmd.prop3 = self.max_values[2]
        if self.pwm_cmd.prop4 > self.max_values[3]:
            self.pwm_cmd.prop4 = self.max_values[3]
        if self.pwm_cmd.prop1 < self.min_values[0]:
            self.pwm_cmd.prop1 = self.min_values[0]
        if self.pwm_cmd.prop2 < self.min_values[1]:
            self.pwm_cmd.prop2 = self.min_values[1]
        if self.pwm_cmd.prop3 < self.min_values[2]:
            self.pwm_cmd.prop3 = self.min_values[2]
        if self.pwm_cmd.prop4 < self.min_values[3]:
            self.pwm_cmd.prop4 = self.min_values[3]

        # Updating the previous error 
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]	

        # ------------------------------------------------------------------------------------------------------------------------
        #Publishing the calculated errors

        self.pwm_pub.publish(self.pwm_cmd)

        #The below Publications can be uncommented during r,p,y tuning 

        #Error Publications->
        # self.zero_error_pub.publish(0)
        # self.roll_error_pub.publish(self.error[0])
        # self.pitch_error_pub.publish(self.error[1])
        # self.yaw_error_pub.publish(self.error[2])

#-------------------------------------------------------------------------------------------------------------
# Main Driver Function

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the PID function recursively until rospy is not shutdown
if __name__ == '__main__':
    #Creating instance of class Edrone
    e_drone = Edrone()

    #assigning Rate  
    r = rospy.Rate(1/e_drone.sample_time)  
    while not rospy.is_shutdown():      
        # calling the PID controller function when rospy is active
        e_drone.pid()
        r.sleep()