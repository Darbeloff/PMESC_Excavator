#!/usr/bin/python
'''
@author: weitung
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
    rospy.init_node('image_transform_node', anonymous=True)
    sub_Cal   = rospy.Subscriber("cal_Data", JointState, cb_CalValue)
    sub_Img = rospy.Subscriber('UBC_Image', Image, cb_Img)
    f = open(os.path.dirname(os.path.realpath(__file__)) +"/" + rospy.get_param("~Filename"), "r")
    s = f.readlines()
    split_array = decodeCalFile(s)
    #split_array = [string.split() for string in s]
    print(split_array.shape)
    #angle_map = angle_map.astype(float)
    #angle_map[0] = float(angle_map[0])
    #print(angle_map[:,1])

def cb_CalValue(msg):
    global BucketPosition
    BucketPosition = msg.position[0]

def cb_Img(msg):
    global angle_map, BucketPosition
    map_l = angle_map.copy()
    Bucket_l = BucketPosition
    try:
        Img_cv = decodeMsg2Img(msg)             #decode ROS messages to Image Matrix
        #print(Img_cv.shape)                    #DEBUG print: see the shape of the image

        if abs(Bucket_l) > 2*pi:
            if Bucket_l > 0:
                Bucket_l -= 2*pi
            else:
                 Bucket_l += 2*pi
        print("Bucket_l: "+str(Bucket_l))
        map_l[:,0] = abs(map_l[:,0]-Bucket_l)   #find which angle on the angle map is the closest to the current angle
        a = map_l[np.argmin(map_l[:,0])]
        b = map_l[np.argsort(map_l[:,0])[-1]]
        print(a)
        print(b)
        
        pts_before = np.float32([[a[0],a[1]],[a[2],a[3]],[a[4],a[5]],[a[6],a[7]]])
        pts_after = np.float32([[0,0],[0,200],[200,200],[200,0]])
        trans_matrix = cv2.getPerspectiveTransform(pts_before,pts_after)
        res = cv2.warpPerspective(Img_cv,trans_matrix,(200,200))

        cv2.imshow("cropped", res)
        cv2.waitKey(1)

        pub_Img = rospy.Publisher('Crop_Image', Image, queue_size=1)        #publish the message Crop_Image and send it to the optical flow
        pub_Img.publish(CvBridge().cv2_to_imgmsg(res, "bgr8"))

    except:
        print('Could not update')    

def decodeMsg2Img(msg):
    Img_cv = []
    bridge = CvBridge()
    image_decoded = bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")   #decode ROS messages to opencv format
    Img_cv = cv2.transpose(image_decoded)
    return Img_cv

def decodeCalFile(s):
    output = []
    for string in s:
        split = string.split("\t")
        out_single = []
        for sq in split:
            if '[' in sq:
                o = sq.replace("[","")
                o = o.replace("]","")
                sq = o.split()
                ret = []
                for num in sq:
                    ret.append(float(num))
            else:
                try:
                    ret = float(sq)
                except:
                    print(sq)
                    ret = 0
            out_single.append(ret)
        output.append(out_single)

    return np.array(output)

if __name__ == '__main__':
    image_crop()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()



