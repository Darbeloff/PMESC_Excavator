#!/usr/bin/python
'''
@author: fes
'''

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import exp_excavator.msg.JointValues as JointVals
import numpy as np

class CalibratorWithIMU:

    def __init__(self):
        rospy.init_node('CalibratorWithIMU', anonymous=True)
        self.rate = 100 #[Hz]
        self.ImuBoom = Imu()
        self.ImuArm  = Imu()
        self.Pos     = JointVals()
        self.CalJointMsg = JointVals()
        
        
        self.P_old = 5
        self.P_new = []
        
        self.posBoom   = []
        self.posArm    = []
        self.posBucket = []
        self.velBoom   = []
        self.velArm    = []
        self.velBucket = []        
        
        self.alpha = 0.0
        self.beta  = 0.0
        
        self.sub_IMUBoom = rospy.Subscriber('IMUBoom' , Imu       , self.cb_IMUBoom);
        self.sub_IMUArm  = rospy.Subscriber('IMUArm'  , Imu       , self.cb_IMUArm);
        self.sub_pos     = rospy.Subscriber('JointPos', JointVals , self.cb_pos);
        self.sub_posDYNA = rospy.Subscriber('JointsDYNA', JointState, self.cb_posDYNA);
        self.pub_JointCalib = rospy.Publisher('JointCalibration', JointVals, queue_size=10);


    
    def cb_IMUBoom(self, msg):
        try:
            self.ImuBoom = msg
        except :
            print("ERRORcalIMU1") 

    def cb_IMUArm(self, msg):
        try:
            self.ImuArm = msg
        except :
            print("ERRORcalIMU2")          
            
    def cb_pos(self, msg):
        try:
            self.posBoom = msg.boom
            self.posArm  = msg.arm
        except :
            print("ERRORcbpos")  
    
    def cb_posDYNA(self, msg):
        try:
            self.posBucket = msg.position[0]
            self.velBucket = msg.velocity[0]
        except :
            print("ERRORcbDYNA")  

    def flow_update(self):

        gamma = np.arctan2(-self.ImuBoom.linear_acceleration.z,self.ImuBoom.linear_acceleration.y)
        delta = np.arctan2(-self.ImuArm.linear_acceleration.z,self.ImuArm.linear_acceleration.y)
               
        self.alpha = np.float64(self.alpha)  +  np.divide(np.float64(self.P_old),(1+np.float64(self.P_old)))*(-(np.float64(gamma)+np.float64(self.posBoom)+np.pi/2) -self.alpha)
        self.beta =  np.float64(self.beta)   +  np.divide(np.float64(self.P_old),(1+np.float64(self.P_old)))*(np.float64(gamma)-np.float64(delta)-np.float64(self.posArm) -self.beta)
        self.P_new  = np.float64(self.P_old) -  np.divide(np.float64(self.P_old)*np.float64(self.P_old),1+np.float64(self.P_old))
        self.P_old = np.array(self.P_new)

        
    def Calibrate_update(self):
        rospy.sleep(3.0)
        r = rospy.Rate(self.rate)
        start_time = rospy.Time.now()
        d = rospy.Duration.from_sec(4)       
        
        while ((rospy.Time.now() - start_time) < d):
            self.flow_update()
            r.sleep()


        self.CalJointMsg =JointState()

        self.CalJointMsg.boom.append(np.float32(self.alpha))
        self.CalJointMsg.arm.append(np.float32(self.beta))
        self.CalJointMsg.bucket.append(np.float32(np.pi))
        self.CalJointMsg.swing.append(0.0)
        
        self.pub_JointCalib.publish(self.CalJointMsg)

             
if __name__ == '__main__':
    CAL = CalibratorWithIMU()
    try:
        CAL.Calibrate_update()
    except KeyboardInterrupt:
        print("Shutting down")

