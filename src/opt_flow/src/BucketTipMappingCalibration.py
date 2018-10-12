#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage, JointState
from opencv_apps.msg import FlowArrayStamped
from opencv_apps.msg import Flow
from opencv_apps.msg import Point2D
from std_msgs.msg import Float32

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import os
import video
from common import anorm2, draw_str
from time import clock
from time import sleep 

CALIBRATE_MODE = 1  #0: Manual Calibration; 1: Use Corner Detection Method; 2: high pass to see the edges; 3: fflow

class BucketTipMapping:
    def __init__(self):
        rospy.init_node('BucketTipMappingNode', anonymous=True)
        self.sub_Img = rospy.Subscriber('UBC_Image', Image, self.cb_Img)                    #Subscribe the image Raw from the bucket video ros bag
        self.sub_Cal   = rospy.Subscriber("cal_Data", JointState, self.cb_CalValue)         #Subscribe the Joint State
        self.sub_Flow = rospy.Subscriber('Crop_Flow', FlowArrayStamped, self.cb_Flow)
        self.calfile = open(os.path.dirname(os.path.realpath(__file__)) +"/CalibrationFile.txt", "w")    #open a file to write calibration data

        # Initiate Necessary Parameters
        self.rate = 100                 #[Hz] the spin speed of the ros

        self.FlowArray = []
        self.BucketPosition = 0.0
        self.RawImage = []

        ##For Manual Calibration 
        self.flag = False
        self.refPt = []
        self.index = 0

        ##For Auto Calibration
        self.first = True

    def cb_Flow(self, msg):
        self.FlowArray = msg.flow

    def cb_CalValue(self, msg):
        self.BucketPosition = msg.position[2]

    def cb_Img(self, msg):
        bridge = CvBridge()
        image_decoded = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  #decode ROS messages to opencv format
        self.RawImage = cv2.transpose(image_decoded)
        #gray = cv2.cvtColor(cv2.transpose(image_decoded),cv2.COLOR_BGR2GRAY)
        #self.RawImage = np.float32(gray)
    
    def click_and_crop(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPt = [(x, y)]

        elif event == cv2.EVENT_LBUTTONUP:
            self.refPt.append((x, y))
            cv2.line(self.RawImage, self.refPt[0], self.refPt[1], (0, 255, 0), 2)
            cv2.imshow("image", self.RawImage)
            self.flag = True

    def findNearestCornerDetection(self, est_arr, detect_arr):
        detection_coor = []
        for coor in est_arr:
            distance_arr = []
            for detect in detect_arr:
                distance_arr.append((detect[0] - coor[0])**2 + (detect[1] - coor[1])**2)
            detection_coor.append(distance_arr[np.argmin(distance_arr)])
        return detection_coor


    # filtering function (low-pass / high-pass
    def filter(self, X, lo=True, x_om_c=0.05, y_om_c=None):
        """
        Given a numpy array containing DFT coefficients, where (Kx=0, Ky=0) is in
        the center of the array, return a filtered version of the DFT coefficients.

        If lo==True, we will keep only the low frequency content (i.e.,
        coefficients below the specified cutoff frequency); otherwise, we do the
        opposite.

        x_om_c is a cutoff, such that the actually frequency cutoff Omega_x is
        x_om_c times 2pi.

        y_om_c is the same, but in the y direction.  If y_om_c is not specified, it
        is assumed to be the same as x_om_c.
        """
        if y_om_c is None:
            y_om_c = x_om_c
        X2 = X#np.array(X)
        nr, nc = X2.shape
        xkc = round(x_om_c * nc)
        ykc = round(y_om_c * nr)
        for r in range(nr):
            for c in range(nc):
                keep = ((r-nr//2)**2/ykc**2 + (c-nc//2)**2/xkc**2) < 1
                if not lo:
                    keep = not keep
                X2[r, c] *= keep
        return X2


    def update(self):
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            if self.RawImage.size > 0:
                if CALIBRATE_MODE == 0:
                    """
                    Use mouse call back to get the tips of the buckets
                    """
                    
                    cv2.imshow("image", self.RawImage)
                    cv2.setMouseCallback("image", self.click_and_crop)
                    cv2.waitKey(0)
                    
                    if self.flag:
                        self.index += 1
                        self.calfile.write(str(self.refPt[0][0]) + " " + str(self.refPt[0][1]) + " " +str(self.refPt[1][0]) + " "+\
                            str(self.refPt[1][1]) + "  " + str(self.BucketPosition) + "  \n")
                        self.calfile.flush()
                        self.flag = False

                if CALIBRATE_MODE == 1:
                    """
                    Use corner detection to find the corners of the bucket
                    After selecting the first four corners of the bucket, Use optical flow to 
                    track the motion of that pixel and estimate the next corner of the bucket
                    """
                    gray = cv2.cvtColor(self.RawImage,cv2.COLOR_BGR2GRAY)
                    inp = np.float32(gray)
                    dst = cv2.cornerHarris(inp,5,7,0.04)  # Find the corners
                    dst = cv2.dilate(dst,None)                      # result is dilated for marking the corners, not important
                    self.RawImage[dst>0.01*dst.max()]=[0,0,255]     # Threshold for an optimal value, it may vary depending on the image.
                    cv2.imshow('dst',self.RawImage)                 # May Use clone in the future to clone another image
                    cv2.waitKey(0)
                    
                    '''
                    if self.first:
                        self.first = False
                        cv2.imshow("image", self.RawImage)
                        cv2.setMouseCallback("image", self.click_and_crop)
                        cv2.waitKey(0)
                    '''

                if CALIBRATE_MODE == 2:
                    gray = cv2.cvtColor(self.RawImage,cv2.COLOR_BGR2GRAY)
                    inp = np.float32(gray)
                    fft_image = np.fft.fft2(inp)
                    fft_image = np.fft.fftshift(fft_image)
                    high_freq = self.filter(fft_image, lo=False, x_om_c=0.25)
                    high_freq = np.fft.ifftshift(high_freq)
                    d_shift = np.array(np.dstack([high_freq.real,high_freq.imag]))
                    img_back = np.fft.ifft2(d_shift)
                    img = cv2.magnitude(img_back[:,:,0],img_back[:,:,1])
                    cv2.imshow('edge',img)

                if CALIBRATE_MODE == 3:
                    pass

                    

            r.sleep()

if __name__ == '__main__':
        bmapping = BucketTipMapping()
        rospy.sleep(0.2)
        try:
            bmapping.update()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()