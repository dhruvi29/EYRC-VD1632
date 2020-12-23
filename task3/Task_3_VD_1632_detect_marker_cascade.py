#!/usr/bin/env python2

# importing required libraries
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from vitarana_drone.msg import *
from numpy import * 
import rospy 
from sensor_msgs.msg import LaserScan

# initializing global variables
x, y, w, h = 0.0,0.0,0.0,0.0

# class for marker detection
class detect():
	# constructor
	def __init__(self):
		# initialising node
		rospy.init_node('detect_marker_cascade', anonymous=True)

		self.img_width = 400
		self.hfov_rad = 1.3962634
		self.focal_length = (self.img_width/2)/tan(self.hfov_rad/2)	

		# publications	
		self.detect_pub = rospy.Publisher("/marker_data", MarkerData, queue_size =1)
		self.detected = rospy.Publisher("/isDetected",Bool,queue_size=1)
		self.image_pub = rospy.Publisher("marker_image",Image,queue_size=1) 

		# initialsing instance variables
		self.img = empty([]) 
		self.bridge = CvBridge()
		self.markerdata = MarkerData()
		self.markerdata.err_x_m = 0.0
		self.markerdata.err_y_m = 0.0
		self.markerdata.marker_id = 1
		self.Center = self.img_width/2
		self.Z_m = 0.0

		# Creating instance of cascade classifier
		self.logo_cascade = cv2.CascadeClassifier('/home/dhairya/catkin_ws/src/vitarana_drone/intro_cascade_classifiers_training_and_usage/data/cascade.xml')

		# subscriptions
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) 
		rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)
		rospy.Subscriber('/marker_ID', Int32, self.marker_ID_callback)

	# callback functions
	def range_finder_bottom_callback(self, msg):
		#assigning distance to obstacles as gotten by range_finder_bottom
		self.Z_m = msg.ranges[0]
		
	def image_callback(self, data):
		try:
			# Converting the image to OpenCV standard image
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") 

		except CvBridgeError as e:
			print(e)

		# showing drone camera frame/feed	
		cv2.imshow("Image window", self.img)
		cv2.waitKey(3)
			
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
		except CvBridgeError as e:
			print(e)

		# changing image to grayscale	
		gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

		# final classification
		logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
		try :

			for (x, y, w, h) in logo:
				cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
			# calculating centre pixel of marker	
			centre_x_pixel= (2*x+w)/2.0
			centre_y_pixel= (2*y+h)/2.0

			# calculating errors
			self.markerdata.err_x_m = (self.Center-centre_x_pixel)*self.Z_m/self.focal_length
			self.markerdata.err_y_m = (self.Center-centre_y_pixel)*self.Z_m/self.focal_length
			self.detected.publish(True)
			
		except Exception as e :
			self.detected.publish(False)
		
		# publishing on marker_data topic
		self.detect_pub.publish(self.markerdata)

	def marker_ID_callback(self,msg):
		self.markerdata.marker_id=msg.data

# main driver code
if __name__ == '__main__':
	detect_obj = detect()
	rospy.spin()