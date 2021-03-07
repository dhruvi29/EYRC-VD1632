#!/usr/bin/env python2
'''
# Team ID:          VD#1632
# Theme:            Vitarana Drone(VD)
# Author List:      Karthik Swaminathan, Dhruvi Doshi, Ninad Jangle, Dhairya Shah
# Filename:         qr_detect.py
# Functions:        image_callback
# Global variables: q, qr_data
'''

# importing required libraries
from sensor_msgs.msg import Image
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import pyzbar.pyzbar as pyzbar
import cv2
import numpy as np
import rospy
from vitarana_drone.msg import *

# global variables 
# q: variable to store qr decoded data
q = ""
# qr_data: initialising qr data as a empty list 
qr_data = list()

class image_proc():
	def __init__(self):
		'''
        Purpose:
        ---
        This function is the constructor for instance of image_proc() class. It initialises all the variables 
        related to the instance and contains publisher and subscriber objects. 

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
		rospy.init_node('barcode_test', anonymous=True) #Initialise rosnode
		# publishers to qr_image, /qrscan 
		self.image_pub = rospy.Publisher("qr_image",Image,queue_size=1) 
		self.qr_pub = rospy.Publisher("/qrscan", qr_code, queue_size =1)
		# Subscriber to /edrone/camera/image_raw
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) 
		
		# self.img: This will contain image frame from camera
		self.img = np.empty([]) 
		# self.bridge: CV Bridge Classifier Objecct
		self.bridge = CvBridge()
		# self.qr_val: qr code object
		self.qr_val = qr_code()

		# qr decoded latitude, longitude, altitude initialised variables
		self.qr_val.latitude = 0.0
		self.qr_val.longitude = 0.0
		self.qr_val.altitude = 0.0
		
	def image_callback(self, data):
		'''
        Purpose:
        ---
       	Callback function for camera topic

        Input Arguments:
        ---
		data: Raw Data from scanned QR code

        Returns:
        ---
        None

        Example call:
        ---
        self.image_callback(data)
        '''
		try:
			# self.img: Converting the image to OpenCV standard image
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") 
			# QR: decoded qr object
			QR = pyzbar.decode(self.img)
			#print('QR',type(QR))

			# decoding and publishing QR values as long as it is detected in camera frame
			for qr in QR:				
				cv2.imshow("Image window", self.img)
				cv2.waitKey(3)
				q = qr.data.decode('utf-8')
				qr_data = q.split(',')
				qr_data[0] = float(qr_data[0])
				qr_data[1] = float(qr_data[1])
				qr_data[2] = float(qr_data[2])
				# print(qr_data)
				self.qr_val.latitude = qr_data[0]
				self.qr_val.longitude = qr_data[1]
				self.qr_val.altitude = qr_data[2]
				#print(self.qr_val)
				self.qr_pub.publish(self.qr_val) 
				
		except CvBridgeError as e:
			print(e)

		# Displaying the image window	
		cv2.imshow("Image window", self.img)
		cv2.waitKey(3)
		
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
		except CvBridgeError as e:
			print(e)

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To create image_proc instance and publish qr data until rospy node shutdowns.
if __name__ == '__main__':
	image_proc_obj = image_proc()
	rospy.spin()