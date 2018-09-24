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

class SpeedCommanderTeleop:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        
        self.rate = 100 #[Hz]
        
        self.joy_val = cmsg.JointValues()

        self.enabled = False
       
        self.joy_switch_servo = []
        self.joy_switch_impedance = []
        self.joy_switch_trajectory = []
        self.arm_motor_position_ref = 0.0

        self.boom_mode = 3 
        self.arm_mode  = 3

        self.buffer_length = 100

        self.power_buffer = np.zeros(self.buffer_length)
        self.regressor_buffer = np.zeros(self.buffer_length)
        self.power_gradient = 0 
        self.power_gradient_last = 0;
        self.velocity_adaptation_last = 0;
        self.velocity_adaptation= 0;

        self.time_switch_last = rospy.get_rostime()

        self.sub_joy = rospy.Subscriber('joy_values', cmsg.JointValues,
                                        self.cb_joy)

        self.sub_joy_right = rospy.Subscriber('joy_right', smsg.Joy,
                                              self.cb_joy_right)

        self.sub_state_arduino = rospy.Subscriber('machine_state_arduino',
                                cmsg.JointStateMachineArduino, self.cb_state_arduino)

        self.pub_arduino   = rospy.Publisher('arduino_commands', cmsg.JointCommandArduino,
                                           queue_size=10)

        self.pub_dynamixel = rospy.Publisher('dynamixel_commands', cmsg.JointCommandDynamixel,
                                           queue_size=10)
        self.pub_gradient      = rospy.Publisher('power_gradient', Float32, queue_size=10)

        self.pub_power     = rospy.Publisher('power', Float32, queue_size=10)

    def cb_joy(self, msg):
        self.joy_val = msg
        
    def cb_state_arduino(self, msg):

        self.arm_motor_current  = msg.armI 
        self.arm_motor_velocity = msg.armV 
        self.arm_motor_position = msg.armP 

        self.boom_motor_position = msg.boomP
        self.boom_motor_velocity = msg.boomV

        self.power_buffer = np.roll(self.power_buffer,1)
        self.power_buffer[0] = self.arm_motor_velocity*self.arm_motor_current
        
        self.regressor_buffer = np.roll(self.regressor_buffer,1)
        self.regressor_buffer[0] = self.boom_motor_velocity

                  
    def cb_joy_right(self, joy):
        self.joy_switch_servo = joy.buttons[1]
        self.joy_switch_impedance = joy.buttons[2]
        self.joy_switch_trajectory = joy.buttons[3]
        #modes for each joint:
            # 0 - position control
            # 1 - impedance simulated mod
            # 2 - velocity ref to microcontroller
            # 3 - disabled
            # 4 - automatic digging

        if self.joy_switch_servo == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(1, 0):
            print('switch enable/disable')
            
            if self.enabled==True:
                self.boom_mode = 3 
                self.arm_mode  = 3
            elif self.enabled==False:
                self.arm_motor_position_ref = self.arm_motor_position
                self.boom_mode = 2 
                self.arm_mode  = 0

            self.enabled = not self.enabled
            self.time_switch_last = rospy.get_rostime()

        if self.joy_switch_impedance == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(1, 0):
            print('switch enable/disable')
            
            if self.enabled==True:
                self.boom_mode = 3 
                self.arm_mode  = 3
            elif self.enabled==False:
                self.boom_mode = 2 
                self.arm_mode  = 1

        if self.joy_switch_trajectory == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(1, 0):
            print('switch enable/disable')
            
            if self.enabled==True:
                self.boom_mode = 3 
                self.arm_mode  = 3
            elif self.enabled==False:
                self.boom_mode = 4 
                self.arm_mode  = 4
            self.enabled = not self.enabled
            self.time_switch_last = rospy.get_rostime()   
    
    def extremum_update(self):
        power_mc     = self.power_buffer     - np.mean(self.power_buffer)
        regressor_mc = self.regressor_buffer - np.mean(self.regressor_buffer)

        A = np.vstack([regressor_mc,np.ones(self.buffer_length)]).T
        self.power_gradient , offset = np.linalg.lstsq(A,power_mc)[0]
        self.velocity_adaptation  = 0.02*(self.power_gradient+self.power_gradient_last)+ self.velocity_adaptation_last    

        #self.pub_gradient.publish( self.velocity_adaptation)
        self.pub_power.publish(self.power_buffer[0])
        self.velocity_adaptation_last = self.velocity_adaptation
        self.power_gradient_last      =  self.power_gradient

    def update(self):
        r = rospy.Rate(self.rate)
        
        
        while not rospy.is_shutdown():
            arduino_controller_msg   = cmsg.JointCommandArduino()
            dynamixel_controller_msg = cmsg.JointCommandDynamixel()

            

            if self.arm_mode == 1:
                self.extremum_update()
                arduino_controller_msg.boomV   = self.joy_val.boom*100.0  + 2.5*self.power_gradient + 0.5*self.velocity_adaptation
            else:
                arduino_controller_msg.boomV   = self.joy_val.boom*100.0
            
            arduino_controller_msg.armV    = self.joy_val.arm*(-100.0)

            if self.arm_mode == 0:
                self.arm_motor_position_ref=self.arm_motor_position_ref + 0.5*self.joy_val.arm
                arduino_controller_msg.armP = self.arm_motor_position_ref

            dynamixel_controller_msg.bucketV    = self.joy_val.bucket*1.0
     
            arduino_controller_msg.BoomMode = self.boom_mode
            arduino_controller_msg.ArmMode =  self.arm_mode

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
