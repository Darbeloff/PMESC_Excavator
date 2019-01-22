#!/usr/bin/python

import rospy
import copy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage, JointState, PointCloud2
from sensor_msgs import point_cloud2
from opencv_apps.msg import FlowArrayStamped
from opencv_apps.msg import Flow
from opencv_apps.msg import Point2D
from std_msgs.msg import Float32
from scipy.interpolate import Rbf

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import os
import video
from common import anorm2, draw_str
from time import clock
from time import sleep

CALIBRATE_MODE = 0  #0: Manual Calibration; 1: Use Corner Detection Method; 2: high pass to see the edges; 3: fflow

class BucketTipMapping:
    """
    Import Note for the users: If you want to calibrate the bucket,
    you must select the corners in the order, top left, bottom left, bottom right, top right
    """
    def __init__(self):
        rospy.init_node('BucketTipMappingNode', anonymous=True)
        self.sub_Img = rospy.Subscriber('UBC_Image', Image, self.cb_Img)                    #Subscribe the image Raw from the bucket video ros bag
        self.sub_Cal   = rospy.Subscriber("cal_Data", JointState, self.cb_CalValue)         #Subscribe the Joint State
        self.sub_pcl = rospy.Subscriber('PointCloud', PointCloud2, self.cb_pcl)   

        if CALIBRATE_MODE == 1: self.sub_Flow = rospy.Subscriber('Crop_Flow', FlowArrayStamped, self.cb_Flow)

        self.calfile = open(os.path.dirname(os.path.realpath(__file__)) +"/CalFile3.txt", "w")    #open a file to write calibration data

        # Initiate Necessary Parameters
        self.rate = 1000                 #[Hz] the spin speed of the ros

        self.FlowArray = []
        self.BucketPosition = 0.0
        self.RawImage = np.array([])
        self.rawImageLocal = []

        ##For Manual Calibration 
        self.flag = False
        self.refPt = []
        self.index = 0

        ##For Auto Calibration
        self.first = True

    def cb_Flow(self, msg):
        self.FlowArray = msg.flow

    def cb_CalValue(self, msg):
        self.BucketPosition = msg.position[0]
        print(self.BucketPosition)

    def cb_Img(self, msg):
        bridge = CvBridge()
        image_decoded = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  #decode ROS messages to opencv format
        self.RawImage = cv2.transpose(image_decoded)
        #gray = cv2.cvtColor(cv2.transpose(image_decoded),cv2.COLOR_BGR2GRAY)
        #self.RawImage = np.float32(gray)

    def cb_pcl(self, cloud_msg):
        cloud_points = list(point_cloud2.read_points(cloud_msg, skip_nans=False, field_names = ("x", "y", "z")))    #read point cloud points to a list
        c_array_reshaped = self.list2array(cloud_points, cloud_msg.height, cloud_msg.width)                              #change 1D data to 2D
        self.pcl = cv2.transpose(c_array_reshaped)
        #cv2.imshow("pc", Img_cv)
        #cv2.waitKey(1)

    def list2array(self, onedlist, height, width):
        return np.array([[onedlist[j+i*width] for j in range(width)] for i in range(height)])

    def writeToFile(self, bucketposition, grid):
        """
        This write the bucket position and the grid to the calibration file
        bucket position is the first entry and the it is concatenated in row - row - row
        """
        rows = grid.shape[0]
        cols = grid.shape[1]
        s = str(bucketposition)+"\t"
        for y in range(rows):
            for x in range(cols):
                s += str(grid[y][x]) + "\t"
        s+="\n"
        self.calfile.write(s)
        self.calfile.flush()

    
    def click_and_crop(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            try:
                self.refPt.append((x, y))
            except:
                self.refPt = [(x, y)]

            cv2.circle(self.pclLocal,(x, y), 4, (0,0,255), -1)
            cv2.imshow("pcl_x", self.pclLocal)
            cv2.waitKey(1)

            if len(self.refPt) == 4:
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
                    if len(self.refPt) == 0:
                        self.rawImageLocal = np.copy(self.RawImage)
                        self.pclLocal = np.copy(self.pcl)
                        bucketPositionLocal = self.BucketPosition
                    
                    #cv2.waitKey(10)
                    pcl_clone = copy.deepcopy(self.pclLocal)
                    pcl_clone = np.nan_to_num(pcl_clone)
                    cv2.imshow("pcl_x", self.pclLocal)
                    #cv2.imshow("pcl_y", self.pclLocal[:,:,1])
                    #cv2.imshow("pcl_z", self.pclLocal[:,:,2])
                    #cv2.imshow("image", self.rawImageLocal)
                    cv2.setMouseCallback("pcl_x", self.click_and_crop)
                    cv2.waitKey(0)
                    
                    if self.flag:

                        corners_array = [[self.refPt[0][0],self.refPt[0][1]],\
                            [self.refPt[1][0],self.refPt[1][1]],[self.refPt[2][0],self.refPt[2][1]],[self.refPt[3][0],self.refPt[3][1]]]

                        # Find the transformation between the square and the image
                        pts_before = [[0,0],[0,10],[10,10],[10,0]]
                        pts_after = corners_array
                        trans_matrix = cv2.getPerspectiveTransform(np.float32(pts_before),np.float32(pts_after))
                        trans_matrix_inv = cv2.getPerspectiveTransform(np.float32(pts_after),np.float32(pts_before))

                        x, y, d = getRBFInputs(np.array([corners_array], dtype=np.int32), pcl_clone)
                        
                        # I swapped rbf_x and rbf_y because the orientation of the
                        # camera and what I defined is different
                        rbf_x = Rbf(x[::8], y[::8], d[::8,1])
                        rbf_y = Rbf(x[::8], y[::8], d[::8,0])
                        rbf_z = Rbf(x[::8], y[::8], d[::8,2])

                        xi, yi = findRBFNodes(10,10,trans_matrix)
                        """
                        value_x = []
                        value_y = []
                        value_z = []
                        for y_node in yi:
                            for x_node in xi:
                                dx, dy, dz = calculateRBF(x_node, y_node, 5, self.pclLocal)
                                value_x.append(dx)
                                value_y.append(dy)
                                value_z.append(dz)
                        """
                        value_x = rbf_x(xi, yi)
                        value_y = rbf_y(xi, yi)
                        value_z = rbf_z(xi, yi)

                        rbfout = getRBFOutputs(value_x, value_y, value_z, 10, 10)
                        print(rbfout.shape)
                        
                        res = cv2.warpPerspective(pcl_clone,trans_matrix_inv,(10,10))
                        cv2.imshow("res", res)
                        cv2.waitKey(1)

                        cv2.imshow("out_x", rbfout)
                        cv2.waitKey(1)

                        self.writeToFile(bucketPositionLocal, rbfout)
                        """
                        self.calfile.write(str(self.refPt[0][0]) + "\t" + str(self.refPt[0][1]) + "\t" +str(self.refPt[1][0]) + "\t"+\
                            str(self.refPt[1][1]) + "\t" + str(self.refPt[2][0]) + "\t" + str(self.refPt[2][1]) + "\t" +\
                            str(self.refPt[3][0]) + "\t"+ str(self.refPt[3][1]) + "\t" + str(bucketPositionLocal) + "\t")
                        self.calfile.flush()
                        """
                        self.refPt = []
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


def findRBFNodes(size_x, size_y, transform):
    """
    x = findRBFNodes(2, 2, np.array([[1,0], [0,1]]))
    print(x)
    """
    x = np.linspace(0, size_x, size_x+1)
    y = np.linspace(0, size_y, size_y+1)

    for yn in y:
        for xn in x:
            try:
                out = np.vstack((out,(np.matmul(transform,np.array([ [xn],[yn],[1] ]))).T))
            except:
                out = (np.matmul(transform,np.array([[xn],[yn],[1]]))).T

    print(out[:,1].flatten())
    return out[:,0], out[:,1]

def calculateRBF(x_node, y_node, offset, img):
    img_clone = copy.deepcopy(img)
    xlow = int(x_node-offset)
    xhigh = int(x_node+offset+1)
    ylow = int(y_node-offset)
    yhigh = int(y_node+offset+1)

    x_indices = []
    y_indices = []
    for y in range(ylow, yhigh):
        for x in range(xlow, xhigh):
            x_indices.append(x)
            y_indices.append(y)

    x_indices = np.array(x_indices)
    y_indices = np.array(y_indices)

    val = img_clone[x_indices, y_indices, :]
    val = np.nan_to_num(val)

    r1 = Rbf(x_indices,y_indices,val[:,0])
    r2 = Rbf(x_indices,y_indices,val[:,1])
    r3 = Rbf(x_indices,y_indices,val[:,2])

    return r1(x_node, y_node), r2(x_node, y_node), r3(x_node, y_node)


def getRBFInputs(pts, img):
    """
    >>> x = getRBFInputs(np.array([[[1,1],[10,1],[10,7],[1,2]]], dtype=np.int32), np.zeros((10,10,10)))
    >>> print(x)
    """
    # blank mask:
    mask = np.zeros_like(img)

    #pts = np.array(pts, dtype=np.int32)
    # filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, pts, 255)
    indices = np.where(mask != 0)
    #print(indices[1][0:50])
    values = img[indices[0],indices[1],:]
    delete_index = np.where(values == 0.0)
    val = np.delete(values, delete_index, 0)
    x = np.delete(indices[0], delete_index, None)
    y = np.delete(indices[1], delete_index, None)
    #print(values.shape)

    return x,y,val

def getRBFOutputs(dx, dy, dz, size_x, size_y):
    """
    >>> x = getRBFOutputs([1,2,3,25,42,1,4,5,2], [2,3,4,2,3,4,4,6,4], [4,5,6, 7, 8, 9,1,2,3],3,3)
    >>> print(x)
    """
    m = np.min(dz)
    ret = []
    for xidx in range(size_x):
        dum = []
        for yidx in range(size_y):
            i = yidx*size_x+xidx
            dum.append([dx[i], dy[i], dz[i]])
        ret.append(dum)
    return np.array(ret)

def testRBF(xi, yi):
    """
    >>> testRBF(13.5, 12.5)
    """
    x = np.linspace(0, 20, 21)
    y = np.linspace(0, 20, 21)
    d = np.power(x,2)+np.power(y,2)
    rbf = Rbf(x,y,d)
    print("Result: ", rbf(xi, yi))

if __name__ == '__main__':
        import doctest
        doctest.testmod()
        
        
        bmapping = BucketTipMapping()
        rospy.sleep(0.2)
        try:
            bmapping.update()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()
        
        
        