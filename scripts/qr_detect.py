#!/usr/bin/env python2


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''

from sensor_msgs.msg import Image
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import pyzbar.pyzbar as pyzbar
# import pyzbar
import cv2
import numpy as np
import rospy

from vitarana_drone.msg import *


q = ""
qr_data = list()

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test', anonymous=True) #Initialise rosnode
		self.image_pub = rospy.Publisher("qr_image",Image,queue_size=1) 
		self.qr_pub = rospy.Publisher("/qrscan", qr_code, queue_size =1)
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.qr_val = qr_code()
		self.qr_val.latitude = 0.0
		self.qr_val.longitude = 0.0
		self.qr_val.altitude = 0.0
		


	# Callback function of camera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			QR = pyzbar.decode(self.img)
			#print('QR',type(QR))
			for qr in QR:
				
				cv2.imshow("Image window", self.img)
				cv2.waitKey(3)
				q = qr.data.decode('utf-8')
				qr_data = q.split(',')
				qr_data[0] = float(qr_data[0])
				qr_data[1] = float(qr_data[1])
				qr_data[2] = float(qr_data[2])
				#print(qr_data)
				self.qr_val.latitude = qr_data[0]
				self.qr_val.longitude = qr_data[1]
				self.qr_val.altitude = qr_data[2]
				#print(self.qr_val)
				self.qr_pub.publish(self.qr_val) 
				
		except CvBridgeError as e:
			print(e)
		cv2.imshow("Image window", self.img)
		cv2.waitKey(3)
		
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
		except CvBridgeError as e:
			print(e)

if __name__ == '__main__':
	image_proc_obj = image_proc()
	rospy.spin()
