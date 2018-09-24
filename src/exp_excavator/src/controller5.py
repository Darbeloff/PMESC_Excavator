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
        
        self.current_setpoint = 2.5

        self.trajectory_stage = 0 

        self.boom_calibration    = 0 
        self.arm_calibration     = 0
        self.bucket_calibration  = 0
        

        self.buffer_length = 100

        self.power_buffer = np.zeros(self.buffer_length)
        self.regressor_buffer = np.zeros(self.buffer_length)
        self.current_error = 0 
        self.current_error_last = 0;
        self.velocity_adaptation_last = 0;
        self.velocity_adaptation= 0;

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

        #modes for whole machine:
            # 0 - disabled/rest
            # 1 - manual joystick mode
            # 2 - impedance mode tests
            # 3 - trajectory tracking test

        #modes for each joint:
            # 0 - position control
            # 1 - impedance simulated mode
            # 2 - velocity ref to microcontroller
            # 3 - disabled
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
            self.boom_mode = 2 
            self.arm_mode  = 0

            self.enabled = True
            self.time_switch_last = rospy.get_rostime()

        if self.joy_switch_impedance == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0, 100000000):
            print('switch to POWER MAXIMIZING')
            self.current_error = 0 
            self.current_error_last = 0;
            self.velocity_adaptation_last = 0;
            self.velocity_adaptation= 0;
            
            self.robot_mode = 2 
            self.boom_mode = 2 
            self.arm_mode  = 1

            self.enabled = True
            self.time_switch_last = rospy.get_rostime() 

        if self.joy_switch_trajectory == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0, 100000000):
            print('switch to CURL')
            
            self.robot_mode = 3 
            self.arm_motor_position_ref = self.arm_motor_position
            self.boom_mode = 2 
            self.arm_mode  = 0

            self.enabled = True
            self.time_switch_last = rospy.get_rostime()   
        
        if self.joy_switch_reset == 1 and rospy.get_rostime()-self.time_switch_last>rospy.Duration(0, 100000000):
            print('switch to RESET')
            
            self.robot_mode = 4 
            self.arm_motor_position_ref = self.arm_motor_position
            self.boom_mode = 2 
            self.arm_mode  = 0

            self.enabled = True
            self.time_switch_last = rospy.get_rostime()   
    
            
    def extremum_update(self):
        
        self.current_error =  self.current_setpoint - self.arm_motor_current

        self.velocity_adaptation  = 0.02*(self.current_error +self.current_error_last) + self.velocity_adaptation_last    

        #self.pub_gradient.publish( self.velocity_adaptation)
        self.pub_power.publish(self.power_buffer[0])
        self.velocity_adaptation_last = self.velocity_adaptation
        self.current_error_last      =  self.current_error

    def update(self):
        r = rospy.Rate(self.rate)
        #modes for whole robot:
            # 0 - disabled/rest
            # 1 - manual joystick mode
            # 2 - impedance mode tests
            # 3 - trajectory tracking test

        #modes for each joint:
            # 0 - position control
            # 1 - impedance simulated mode
            # 2 - velocity ref to microcontroller
            # 3 - disabled
        
        while not rospy.is_shutdown():
            arduino_controller_msg   = cmsg.JointCommandArduino()
            dynamixel_controller_msg = cmsg.JointCommandDynamixel()

            arduino_controller_msg.BoomMode = self.boom_mode
            arduino_controller_msg.ArmMode =  self.arm_mode
            
            if self.robot_mode == 0:
                arduino_controller_msg.armV    = 0

            elif self.robot_mode == 1:
                #set boom
                arduino_controller_msg.boomV   = self.joy_val.boom*20.0
                
                #set arm
                self.arm_motor_position_ref    = self.arm_motor_position_ref + 0.1*self.joy_val.arm
                arduino_controller_msg.armP    = self.arm_motor_position_ref
                arduino_controller_msg.armV    = self.joy_val.arm*(-20.0)

                #set bucket 
                dynamixel_controller_msg.bucketV    = self.joy_val.bucket*1.0


            elif self.robot_mode == 2:
                #set boom
                self.extremum_update()
                boom_cmd_unclipped = 0.5*(self.velocity_adaptation + self.current_error)
                arduino_controller_msg.boomV   = np.clip(boom_cmd_unclipped,-15.0,15.0)
                
                #set arm
                #simulated dynamics set in microcontroller

                #set bucket 
                dynamixel_controller_msg.bucketV    = self.joy_val.bucket*1.0


            elif self.robot_mode == 3:

                arduino_controller_msg.armV    = 0
                arduino_controller_msg.armP    = self.arm_motor_position_ref

                if (0.02*self.boom_motor_position - self.boom_calibration) > -0.70:
                    arduino_controller_msg.boomV     = -0.0
                else:
                    arduino_controller_msg.boomV     = 0

                if (self.bucket_motor_position) < -1.15:
                    dynamixel_controller_msg.bucketV     = 0.8
                else:
                    dynamixel_controller_msg.bucketV     = 0

            elif self.robot_mode == 5:

                print(0.02*self.arm_motor_position - self.arm_calibration)
                if abs(0.02*self.arm_motor_position - self.arm_calibration  - 0.8 ) > 0.05:
                    if (0.02*self.arm_motor_position - self.arm_calibration) < 0.8:
                        self.arm_motor_position_ref    = self.arm_motor_position_ref + 0.05*1.0
                        arduino_controller_msg.armP    = self.arm_motor_position_ref
                    else:
                        self.arm_motor_position_ref    = self.arm_motor_position_ref - 0.05*1.0
                        arduino_controller_msg.armP    = self.arm_motor_position_ref 

                if abs(0.02*self.boom_motor_position - self.boom_calibration  + 0.50 ) > 0.05:
                    if (0.02*self.boom_motor_position - self.boom_calibration) > -0.50:
                        arduino_controller_msg.boomV     = -8
                    else:
                        arduino_controller_msg.boomV     = 8


                if abs(self.bucket_motor_position + 1.15 ) > 0.05:
                    if (self.bucket_motor_position) < -1.15:
                        dynamixel_controller_msg.bucketV     = 0.7
                    else:
                        dynamixel_controller_msg.bucketV     = -0.7


            else:
                arduino_controller_msg.boomV   = self.joy_val.boom*100.0
                arduino_controller_msg.armV    = self.joy_val.arm*(-100.0)
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
