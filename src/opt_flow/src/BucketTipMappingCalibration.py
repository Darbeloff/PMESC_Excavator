#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float32

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import video
from common import anorm2, draw_str
from time import clock
from time import sleep 

CALIBRATE_MODE = 0 	#0: Manual Calibration; 1: Use Corner Detection Method

class BucketTipMapping:
	def __init__(self):
		rospy.init_node('BucketTipMappingNode', anonymous=True)
		self.sub_Img = rospy.Subscriber('UBC_Image', Image, self.cb_Img)				#Subscribe the image Raw from the bucket video ros bag
		self.sub_Cal   = rospy.Subscriber("cal_Data", JointState, self.cb_CalValue)		#Subscribe the Joint State
		
		# Initiate Necessary Parameters
		self.rate = 100 				#[Hz] the spin speed of the ros

		self.BucketPosition = 0.0
		self.RawImage = []

		##For Manual Calibration 
		self.flag = False
		self.refPt = []
		self.index = 0

	def cb_CalValue(msg):
		self.BucketPosition = msg.position[2]

	def cb_Img(msg):
		bridge = CvBridge()
	    image_decoded = bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")	#decode ROS messages to opencv format
	    gray = cv2.cvtColor(cv2.transpose(image_decoded),cv2.COLOR_BGR2GRAY)
		self.RawImage = np.float32(gray)
	
	def click_and_crop(event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.refPt = [(x, y)]

		elif event == cv2.EVENT_LBUTTONUP:
			self.refPt.append((x, y))
			cv2.line(self.RawImage, refPt[0], refPt[1], (0, 255, 0), 2)
			cv2.imshow("image", self.RawImage)
			self.flag = True

	def update(self):
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():

        	if CALIBRATE_MODE == 0:
        		"""
				Use mouse call back to get the tips of the buckets
				"""
        		cv2.imshow("image", se;f.RawImage)
			 	cv2.setMouseCallback("image", self.click_and_crop)
				cv2.waitKey(0)
				
				if flag:
					self.index += 1
					print(str(msg.header.seq)+" "+str(self.refPt[0][0]) + " " + str(self.refPt[0][1]) + " " +str(self.refPt[1][0]) + " "+str(self.refPt[1][1]) + "  ")
					self.flag = False

            self.pub_arduino.publish(arduino_controller_msg)
            self.pub_dynamixel.publish(dynamixel_controller_msg)
            r.sleep()

if __name__ == '__main__':
    	bmapping = BucketTipMapping()
    	rospy.sleep(0.2)
    	try:
        	bmapping.update()
    	except KeyboardInterrupt:
        	print("Shutting down")
    	cv2.destroyAllWindows()