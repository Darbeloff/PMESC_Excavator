#!/usr/bin/python
'''
@author: fes
'''

import rospy
from sensor_msgs.msg import Image, JointState
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import video
import os
from common import anorm2, draw_str
from time import clock
from time import sleep 
from math import pi

angle_map = []
BucketPosition = 0.0

########################################
## This Node use the calibration data - relating joint angle and the pixel we should be cropping
## The calibration data was gathered by mouse_click_test.py where I manually record the mapping
## between the angle and the pixel values
########################################
def image_crop():
	global angle_map
	rospy.init_node('crop_image_node', anonymous=True)
	sub_Cal   = rospy.Subscriber("cal_Data", JointState, cb_CalValue)
	sub_Img = rospy.Subscriber('UBC_Image', Image, cb_Img)
	f = open(os.path.dirname(os.path.realpath(__file__)) +"/" + rospy.get_param("~Filename"), "r")
	s = f.readlines()[0].split()
	angle_map = np.array(s).reshape(len(s)/5,5)
	angle_map = angle_map.astype(float)
	print(angle_map)

def cb_CalValue(msg):
	global BucketPosition
	BucketPosition = msg.position[2]

def cb_Img(msg):
	global angle_map, BucketPosition
	map_l = angle_map.copy()
	Bucket_l = BucketPosition
        try:
	    	Img_cv = decodeMsg2Img(msg)	#decode ROS messages to Image Matrix
	    	print(Img_cv.shape)			#DEBUG print: see the shape of the image
	    
		    if abs(Bucket_l) > 2*pi:
		    	if Bucket_l > 0:
			     Bucket_l -= 2*pi
			else:
			     Bucket_l += 2*pi
		    map_l[:,4] = abs(map_l[:,4]-Bucket_l)	#find which angle on the angle map is the closest to the current angle
		    a = map_l[np.argmin(map_l[:,4])]
		    print(a)

		    #Evaluate four corners based on current mapping of the angle
		    ymin = int(a[1] - 50) if int(a[1] - 50) > 0 else 0
		    ymax = int(a[1] + 50) if int(a[1] + 50) < Img_cv.shape[0] else Img_cv.shape[0]
		    xmin = int(a[0] - 20) if int(a[0] - 20) > 0 else 0
		    xmax = int(a[2] + 20) if int(a[2] + 20) < Img_cv.shape[1] else Img_cv.shape[1]
		    crop_img = Img_cv[ymin:ymax, xmin:xmax] # ymin ymax xmin xmax
		    cv2.imshow("cropped", crop_img)
		    cv2.waitKey(3)

		    pub_Img = rospy.Publisher('Crop_Image', Image, queue_size=1)		#publish the message Crop_Image and send it to the optical flow
		    pub_Img.publish(bridge.cv2_to_imgmsg(crop_img, "bgr8"))

        except:
            print('Could not update')    

def decodeMsg2Img(msg):
	Img_cv = []
	bridge = CvBridge()
    image_decoded = bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")	#decode ROS messages to opencv format
    Img_cv = cv2.transpose(image_decoded)
    return Img_cv

if __name__ == '__main__':
    image_crop()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

