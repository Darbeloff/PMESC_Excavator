#!/usr/bin/python
'''
@author: weitung
'''

import rospy
from sensor_msgs.msg import Image, JointState, PointCloud2
from sensor_msgs.msg import CompressedImage
from sensor_msgs import point_cloud2
from std_msgs.msg import Float32

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import video
import os
from common import anorm2, draw_str
from time import clock
from time import sleep 
from math import pi, sqrt, isnan
from scipy import interpolate

scale_factor = 0.005			#0.000000005
offset = 0.0
VOLUME_CAL_METHODS = 0 			#0: Add all pixels distance directly, 1: Pyramid method
ENABLE_MASK = True			#Fill in all the undefined pixel values by interpolating their neighbors
norm_max = 0.25

def image_crop():
	#####################################################
	#MAIN FUNCTION:
	#initialize the node and set up all the subscriptions
	#####################################################
	rospy.init_node('crop_image_node1', anonymous=True)				#Initialize the node
	sub_Img = rospy.Subscriber('UBC_Image', CompressedImage, cb_CompImg)		#Subscribe/import the depth image
	sub_rgbImg = rospy.Subscriber('RGB_Image', CompressedImage, cb_RGBImg)		#Subscribe/import the rgb image
	sub_pcl = rospy.Subscriber('PointCloud', PointCloud2, cb_pcl)			#Subscribe/import the point cloud

def cb_pcl(cloud_msg):
	cloud_points = list(point_cloud2.read_points(cloud_msg, skip_nans=False, field_names = ("x", "y", "z")))
	c_array_reshaped = np.array([[cloud_points[j+i*cloud_msg.width] for j in range(cloud_msg.width)] for i in range(cloud_msg.height)])
	
	ymin = rospy.get_param("~pcymin")
	ymax = rospy.get_param("~pcymax")
	xmin = rospy.get_param("~pcxmin")
	xmax = rospy.get_param("~pcxmax")
	Img_cv = cv2.transpose(c_array_reshaped)
	crop_img = Img_cv[ymin:ymax, xmin:xmax]
	crop_img_x = crop_img[:,:,0]
	crop_img_y = crop_img[:,:,1]
	crop_img_z = crop_img[:,:,2]
	
	num_p = crop_img.shape[0]*crop_img.shape[1]
	total_volume = 0

	if (ENABLE_MASK):
		m_x = fill_in_hole(crop_img_x)
		m_y = fill_in_hole(crop_img_y)
		m_z = fill_in_hole(crop_img_z)
	else:
		m_x = crop_img_x
		m_y = crop_img_y
		m_z = crop_img_z

	if (VOLUME_CAL_METHODS == 0):
		crop_img_total = np.sqrt(np.square(m_x) + np.square(m_y) + np.square(m_z))
		total_volume += np.sum(crop_img_total)
		################debug_check_Norm_max#####################	
		#print(np.max(crop_img_total))
		#########################################################
		################debug_print_image########################
		crop_img_total = (1.0-0.0)*(crop_img_total - np.min(crop_img_total))/(np.max(crop_img_total) - np.min(crop_img_total))+0.0
		cv2.imshow("pc_total", crop_img_total)
		#########################################################

	elif (VOLUME_CAL_METHODS == 1):
		for j in range(m_x.shape[0]-1):
		    for i in range(m_x.shape[1]-1):
			vectors = [[m_x[j+1][i],m_x[j][i],m_x[j][i+1]],[m_y[j+1][i],m_y[j][i],m_y[j][i+1]],[m_z[j+1][i],m_z[j][i],m_z[j][i+1]]]
			total_volume += cal_pyramid_volume(vectors)
		################debug_print_image########################
		m_x = abs(m_x)
		m_x *= (1.0-0.0)*(m_x - np.min(m_x))/(np.max(m_x) - np.min(m_x))+0.0
		cv2.imshow("mx_total", m_x)
		#########################################################

	print(str(total_volume/num_p/scale_factor)+"          ")

def fill_in_hole(point_cloud_axis_one_dim_data):
	masked = cal_impaintMaskforBolt(point_cloud_axis_one_dim_data)
	valid_mask = ~np.isnan(masked)
	coords = np.array(np.nonzero(valid_mask)).T
	values = masked[valid_mask]

	it = interpolate.LinearNDInterpolator(coords, values, fill_value=0)

	filled = it(list(np.ndindex(masked.shape))).reshape(masked.shape)
	#####################################debug_amplify##############################################
	#masked = abs(filled)
	#masked *= 255.0/np.max(masked)
	#print(masked)
	#cv2.imshow("mask", masked)
	return filled	

def cal_pyramid_volume(Matrix):
	return abs(np.linalg.det(Matrix))/6.0

def cb_RGBImg(msg):
	np_arr = np.fromstring(msg.data, np.uint8)
        image_decoded = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        Img_cv = cv2.transpose(image_decoded)
	ymin = rospy.get_param("~ymin")
	ymax = rospy.get_param("~ymax")
	xmin = rospy.get_param("~xmin")
	xmax = rospy.get_param("~xmax")
	crop_img = Img_cv[ymin:ymax, xmin:xmax]
	#cv2.imshow("cropped_rgb", crop_img)
	#cv2.waitKey(3)

def cb_CompImg(msg):
	np_arr = np.fromstring(msg.data, np.uint8)
        image_decoded = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        Img_cv = cv2.transpose(image_decoded)
	ymin = rospy.get_param("~ymin")
	ymax = rospy.get_param("~ymax")
	xmin = rospy.get_param("~xmin")
	xmax = rospy.get_param("~xmax")
	crop_img = Img_cv[ymin:ymax, xmin:xmax] # ymin ymax xmin xmax
	#crop_img[indices] = 0
	#num_p = cal_valid_points(crop_img)
	#num_p = crop_img.shape[0]*crop_img.shape[1]
	#m = cal_impaintMask(crop_img)
	#new_img = cv2.inpaint(crop_img,m,30,1)
	#new_img = crop_img
	#raw_volume = np.sum(crop_img)/num_p/scale_factor
	#sand_volume = raw_volume - offset
	#print(sand_volume)
	cv2.imshow("cropped", crop_img)
	cv2.waitKey(3)

def cal_impaintMask(img):
	grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	mask = np.zeros(grey.shape, np.uint8)
	#print(np.where(abs(grey-0.0)<20))
	mask[np.where(abs(grey-0.0)<20)] = 255
	return mask

def cal_impaintMaskforBolt(img):
	ymin = rospy.get_param("~boltymin")
	ymax = rospy.get_param("~boltymax")
	xmin = rospy.get_param("~boltxmin")
	xmax = rospy.get_param("~boltxmax")
	img[ymin:ymax, xmin:xmax] = "nan"
	return img

def cal_valid_points(img):
	l = img.tolist()
	num_valid_points = 0
	for e in l:
	    for k in e:
		if abs(k[0]-0.0) > 0.001:
		    num_valid_points += 1
	return num_valid_points

if __name__ == '__main__':
    image_crop()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
