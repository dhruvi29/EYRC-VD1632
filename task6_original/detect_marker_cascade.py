#!/usr/bin/env python2

'''
# Team ID:          VD 1632
# Theme:            Vitarana Drone(VD)
# Author List:      Karthik Swaminathan, Dhruvi Doshi, Ninad Jangle, Dhairya Shah
# Filename:         attitude_position_controller.py
# Functions:        None (but Callback functions)
# Global variables: marker_id
'''

import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from vitarana_drone.msg import *
from numpy import * 
import rospy 
from sensor_msgs.msg import LaserScan, NavSatFix

# marker_id: Records Id number of the detected marker
marker_id = 1


class detect():
	
	def __init__(self):

		'''
        Purpose:
        ---
        Initializes class variables and set publishers and subscribers

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        This function is a constructor, gets implicity called on creation of instance of class Edrone
        e_drone=detect()
        '''
        # initializing ros node with name attitude_controller
		rospy.init_node('detect_marker_cascade', anonymous=True)

		# self.curr_point: Current point of drone from gps
		self.curr_point = [0,0,0]
		# self.img_width: pixel image width 
		self.img_width = 400
		# self.h_fov_rad: Horizontal Field of View of Camera in Radians
		self.hfov_rad = 1.3962634
		# self.focal_length: constant focal length of camera
		self.focal_length = (self.img_width/2)/tan(self.hfov_rad/2)
		
		# self.img: Feed image array 
		self.img = empty([]) 
		# self.Center: Center coordinate of image
		self.Center = self.img_width/2
		# self.bridge: CV Bridge class instance
		self.bridge = CvBridge()
		
		self.markerdata = MarkerData()
		self.markerdata.err_x_m = 0.0
		self.markerdata.err_y_m = 0.0
		self.markerdata.marker_id = 1
		
		#self.logo_cascade: Cascade Classified Marker Detection object
		self.logo_cascade = cv2.CascadeClassifier('/home/dhruvi/Desktop/catkin_ws/src/vitarana_drone/intro_cascade_classifiers_training_and_usage/data/cascade.xml')

		# --------------------------------------------------------------------------------------
        # Publishing /marker_image, /edrone/err_x_m, /edrone/err_y_m, /marker_data, /isDetected 
		self.image_pub = rospy.Publisher("marker_image",Image,queue_size=1) 
		self.err_xm_pub = rospy.Publisher("/edrone/err_x_m", Float64, queue_size =1)
		self.err_ym_pub = rospy.Publisher("/edrone/err_y_m", Float64, queue_size =1)		
		self.detect_pub = rospy.Publisher("/marker_data", MarkerData, queue_size =1)
		self.detected = rospy.Publisher("/isDetected",Bool,queue_size=1)

        # --------------------------------------------------------------------------------------------
		# subscribing to /edrone/gps, /edrone/camera/image_raw, /edrone/range_finder_bottom, /altitude
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)		
		rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) 
		rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)
		rospy.Subscriber('/altitude', Float32, self.altitude_value)

		
		# ------------------------------------------------------------------------------------------

	# Callback functions 

	#updating distance to obstacles as recieved by range_finder_bottom
	def range_finder_bottom_callback(self, msg):
		self.Z_m = msg.ranges[0]

    # updating current coordinates as per gps
	def gps_callback(self, msg):
		self.curr_point[0] = msg.latitude
		self.curr_point[1] = msg.longitude
		self.curr_point[2] = msg.altitude		

    # updating current altitude as per range_finder_bottom
	def altitude_value(self,msg):
		self.altitude = msg.data
		
	def image_callback(self, data):
		'''
		Purpose:
		---
		Updates camera feed and marker detection

		Input Arguments:
		---
		data: data recieved from topic /edrone/camera/image_raw

		Returns:
		---
		None

		Example call:
		---
		Gets called when topic has a message published
		'''

		# Converting the image to OpenCV standard image
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") 

		except CvBridgeError as e:
			print(e)
		cv2.imshow("Detection window", self.img)
		cv2.waitKey(3)
			

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
		except CvBridgeError as e:
			print(e)

		# Converting bgr image to gray scale
		gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

		# image, reject levels level weights.
		logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)

		# Extracting error of drone from marker
		try :

			for (x, y, w, h) in logo:
		    		cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
			centre_x_pixel= (2*x+w)/2.0
			centre_y_pixel= (2*y+h)/2.0

			self.markerdata.err_x_m = (self.Center-centre_x_pixel)*(self.curr_point[2] - self.altitude)/self.focal_length
			self.markerdata.err_y_m = (self.Center-centre_y_pixel)*(self.curr_point[2] - self.altitude)/self.focal_length
			self.markerdata.marker_id = marker_id

			print(self.markerdata.err_x_m,self.markerdata.err_y_m)
			self.detected.publish(True)
		except Exception as e :
			self.detected.publish(False)

		# Publishing on marker data
		try :
			self.detect_pub.publish(self.markerdata)
			self.err_xm_pub.publish(self.markerdata.err_x_m)
			self.err_ym_pub.publish(self.markerdata.err_y_m)
		except Exception as e :
			pass

	# Incrementing marker id 					
	@classmethod 
	# def increment_marker_id(cls):
	# 	cls.marker_id += 1
	
# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    TO detect marker
if __name__ == '__main__':
	detect_obj = detect()
	rospy.spin()
