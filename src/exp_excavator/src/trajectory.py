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
        self.traj_stage = 0 

        #   stages:
        #   0 - stationary rest
        #   1 - move to initial dig position
        #   2 - pierce and drag
        #   3 - scoop
        #   4 - move to check
        #   5 - check
        #   6 - dump 
        
        self.sub_joy = rospy.Subscriber('joy_values', cmsg.JointValues,
                                        self.cb_joy)

        self.sub_joy_right = rospy.Subscriber('joy_right', smsg.Joy,
                                              self.cb_joy_right)

        self.sub_state_arduino   = rospy.Subscriber('machine_state_arduino',
                                cmsg.JointStateMachineArduino, self.cb_state_arduino)
        
        self.sub_command_arduino = rospy.Subscriber('arduino_commands',
                                cmsg.JointCommandArduino, self.cb_command_arduino)

        self.pub_arduino   = rospy.Publisher('arduino_commands', cmsg.JointCommandArduino,
                                           queue_size=10)

        self.pub_power     = rospy.Publisher('trajectory demand', Float32, queue_size=10)

    def cb_joy(self, msg):
        self.joy_val = msg
        
    def cb_command_arduino(self, msg):

#        self.arm_motor_current  = msg.armI 
#       self.arm_motor_velocity = msg.armV 
#        self.arm_motor_position = msg.armP 

#        self.boom_motor_position = msg.boomP
#        self.boom_motor_velocity = msg.boomV

        self.power_buffer = np.roll(self.power_buffer,1)
        self.power_buffer[0] = self.arm_motor_velocity*self.arm_motor_current
        
        self.regressor_buffer = np.roll(self.regressor_buffer,1)
        self.regressor_buffer[0] = self.boom_motor_velocity
                   
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
