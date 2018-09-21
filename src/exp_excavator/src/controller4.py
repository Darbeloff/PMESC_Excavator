#!/usr/bin/python
'''
Created on Feb 24, 2017

@author: fes
'''

import rospy
import exp_excavator.msg as cmsg
import sensor_msgs.msg as smsg
import math
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class SpeedCommanderTeleop:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        
        self.rate = 100 #[Hz]
        
        self.joy_val = cmsg.JointValues()
        
        #initialise variables
        self.enabled = False
       
        self.joy_switch_servo = []
        self.joy_switch_impedance = []
        self.joy_switch_trajectory = []

        self.boom_motor_position_ref = 0.0
        self.arm_motor_position_ref = 0.0

        self.boom_motor_velocity_ref = 0.0
        self.arm_motor_velocity_ref = 0.0

        self.robot_mode = 0 

        self.boom_mode = 3 
        self.arm_mode  = 3
       
        self.boom_calibration    = 0 
        self.arm_calibration     = 0
        self.bucket_calibration  = 0

        self.boom_motor_position = 0
        self.arm_motor_position = 0

        self.time_switch_last = rospy.get_rostime()
        self.time_switch_last_trajectory = rospy.get_rostime()


        self.sub_joy = rospy.Subscriber('joy_values', cmsg.JointValues,
                                        self.cb_joy)

        self.sub_joy_right = rospy.Subscriber('joy_right', smsg.Joy,
                                              self.cb_joy_right)

        self.sub_state_arduino = rospy.Subscriber('machine_state_arduino',
                                cmsg.JointStateMachineArduino, self.cb_state_arduino)

        self.sub_pos_dynamixel   = rospy.Subscriber('joint_states_dynamixel', JointState, self.cb_pos_dynamixel);

        self.sub_joint_calibration = rospy.Subscriber('JointCalibration',
                                cmsg.JointCalibration, self.cb_joint_calibration)

        self.pub_arduino   = rospy.Publisher('arduino_commands', cmsg.JointCommandArduino,
                                           queue_size=10)

        self.pub_dynamixel = rospy.Publisher('dynamixel_commands', cmsg.JointCommandDynamixel,
                                           queue_size=10)


    def cb_joy(self, msg):
        self.joy_val = msg
        
    def cb_state_arduino(self, msg):

        self.arm_motor_current  = msg.armI 
        self.arm_motor_velocity = msg.armV 
        self.arm_motor_position = msg.armP 

        self.boom_motor_position = msg.boomP
        self.boom_motor_velocity = msg.boomV


    def cb_pos_dynamixel(self, msg):
        try:
            self.bucket_motor_position = msg.position[0]

        except :
            print("ERRORcbDYNA")  

    def cb_joint_calibration(self, msg):

        self.boom_calibration    = msg.boom
        self.arm_calibration     = msg.arm
        self.bucket_calibration  = msg.bucket
                  
    def cb_joy_right(self, joy):
        self.joy_switch_stop       = joy.buttons[0]
        self.joy_switch_servo      = joy.buttons[1]
        self.joy_switch_impedance  = joy.buttons[2]
        self.joy_switch_trajectory = joy.buttons[3]
        self.joy_switch_reset      = joy.buttons[4]

    
        if self.joy_switch_stop == 1:
            print('DISABLED')
            self.robot_mode = 0            
            self.boom_mode = 3 
            self.arm_mode  = 3
            self.enabled = False

        if self.joy_switch_servo == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0,100000000):
            print('switch to MANUAL')
            
            self.robot_mode = 1
            self.arm_motor_position_ref = self.arm_motor_position
            self.boom_motor_position_ref = self.boom_motor_position
            self.boom_mode = 0
            self.arm_mode  = 0

            self.enabled = True
            self.time_switch_last = rospy.get_rostime()



    def update(self):
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            arduino_controller_msg   = cmsg.JointCommandArduino()
            dynamixel_controller_msg = cmsg.JointCommandDynamixel()

            arduino_controller_msg.BoomMode = self.boom_mode
            arduino_controller_msg.ArmMode =  self.arm_mode

           
            if self.robot_mode == 1:
                #set boom
                self.boom_motor_position_ref = self.boom_motor_position_ref + 0.2*self.joy_val.boom
                arduino_controller_msg.boomP = self.boom_motor_position_ref
                arduino_controller_msg.boomV   = self.joy_val.boom*20.0
                
                #set arm
                self.arm_motor_position_ref = self.arm_motor_position_ref + 0.2*self.joy_val.arm
                arduino_controller_msg.armP = self.arm_motor_position_ref
                arduino_controller_msg.armV    = self.joy_val.arm*(20.0)
                
                #set bucket 
                dynamixel_controller_msg.bucketV    = self.joy_val.bucket*1.0

            self.pub_arduino.publish(arduino_controller_msg)
            self.pub_dynamixel.publish(dynamixel_controller_msg)
            r.sleep()
            
if __name__ == '__main__':
    sc = SpeedCommanderTeleop()
    rospy.sleep(0.2)
    try:
        sc.update()

    except rospy.ROSInterruptException:
        pass
