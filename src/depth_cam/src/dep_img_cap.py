#!/usr/bin/python
'''
@author: weitung
'''

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
from time import clock
from time import sleep 



def image_crop():
	rospy.init_node('depth_image_cap_node', anonymous=True)
	sub_Img = rospy.Subscriber('Raw_Image', Image, cb_Img)
	

def cb_Img(msg_depth):
    try:
      # The depth image is a single-channel float32 image
      # the values is the distance in mm in z axis
      bridge = CvBridge()
      cv_image = bridge.imgmsg_to_cv2(msg_depth, "32SC1")
      cv_image_array = np.array(cv_image, dtype = np.dtype('f8')) # Convert the depth image to a Numpy array since most cv2 functions require Numpy arrays.
      cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX) # Normalize the depth image to fall between 0 (black) and 1 (white)
      cv_image_resized = cv2.resize(cv_image_norm, (640,480), interpolation = cv2.INTER_CUBIC) # Resize to the desired size
      depthimg = cv_image_resized
      cv2.imshow("Image depth_cam_cap_node", cv_image_norm)
      cv2.waitKey(1)
    except CvBridgeError as e:
      print(e)

'''
        try:
	    Img_cv = []
	    bridge = CvBridge()
        image_decoded = bridge.imgmsg_to_cv2(msg,desired_encoding="32FC1")
        Img_cv = cv2.transpose(image_decoded)
	    ymin = rospy.get_param('~ymin')
	    ymax = rospy.get_param('~ymax')
	    xmin = rospy.get_param('~xmin')
	    xmax = rospy.get_param('~xmax')
	    #crop_img = Img_cv[ymin:ymax, xmin:xmax] # ymin ymax xmin xmax
	    cv2.imshow("cropped", Img_cv)
	    cv2.waitKey(3)
	    #pub_Img = rospy.Publisher('Crop_Image', Image, queue_size=1)
	    #pub_Img.publish(bridge.cv2_to_imgmsg(crop_img, "bgr8"))

        except:
        print('Could not update')    
'''

if __name__ == '__main__':
    image_crop()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

