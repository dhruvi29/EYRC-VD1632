#!/usr/bin/env python2

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
import numpy as np
import rospy

X, Y = 0.0, 0.0
focal_length = 0.0
img_width = 400
hfov_rad = 1.3962634

class detection():

	# Initialise everything
	def __init__(self):
		rospy.init_node('detect_test', anonymous=True) #Initialise rosnode
		self.image_pub = rospy.Publisher("image",Image,queue_size=1) 
		self.err_xm_pub = rospy.Publisher("/edrone/err_x_m", MarkerData, queue_size =1)
		self.err_ym_pub = rospy.Publisher("/edrone/err_y_m", MarkerData, queue_size =1)
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
        	rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()

	def range_finder_bottom_callback(self, msg):
		self.Z_m = msg.ranges[0]

	# Callback function of camera topic
	def image_callback(self, data):
		self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		cv2.imshow("Image window", self.img)
		cv2.waitKey(3)		

		self.logo_cascade = cv2.CascadeClassifier('../intro_cascade_classifiers_training_and_usage/data/cascade.xml')

		img = cv2.imread('../intro_cascade_classifiers_training_and_usage/test_2.png')  # Source image
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		# image, reject levels level weights.
		self.logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)

		for (x, y, w, h) in logo:
    			cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
		plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
		plt.show()

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
		self.err_xm_pub.publish()
		self.err_ym_pub.publish()

	def Error_Minimization(self):
		focal_length = (img_width/2)/tan(hfov_rad/2)
		self.X = centre_x_pixel*Z_m/focal_length
		self.Y = centre_y_pixel*Z_m/focal_length
		

if __name__ == '__main__':
	detection_obj = detection()
	rospy.spin()

