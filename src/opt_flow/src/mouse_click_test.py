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

refPt = []
BucketPos = 0.0
index = 0
Img_cv = []
flag = False

def mouse_click_test():
	rospy.init_node('mouse_click_test_node', anonymous=True)
	sub_Img = rospy.Subscriber('UBC_Image', Image, cb_Img)				#Subscribe the image Raw

def click_and_crop(event, x, y, flags, param):
	global refPt, image, flag
 
	if event == cv2.EVENT_LBUTTONDOWN:
		refPt = [(x, y)]

	elif event == cv2.EVENT_LBUTTONUP:
		refPt.append((x, y))
		cv2.line(Img_cv, refPt[0], refPt[1], (0, 255, 0), 2)
		cv2.imshow("image", Img_cv)
		flag = True


def cb_Img(msg):
	global Img_cv, index, flag
	bridge = CvBridge()
        image_decoded = bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")	#decode ROS messages to opencv format
        Img_cv = cv2.transpose(image_decoded)
	"""
	detect the position of the bucket tips
	"""
	cv2.imshow("image", Img_cv)
 	cv2.setMouseCallback("image", click_and_crop)
	cv2.waitKey(0)
	
	if flag:
		index = index + 1
		print(str(msg.header.seq)+" "+str(refPt[0][0]) + " " + str(refPt[0][1]) + " " +str(refPt[1][0]) + " "+str(refPt[1][1]) + "  ")
		flag = False

if __name__ == '__main__':
    	mouse_click_test()
    	try:
        	rospy.spin()
    	except KeyboardInterrupt:
        	print("Shutting down")
    	cv2.destroyAllWindows()
