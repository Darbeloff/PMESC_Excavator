#!/usr/bin/python
'''
Created on Feb 24, 2017

@author: fes
'''

import rospy
import exp_excavator.msg as cmsg
import sensor_msgs.msg as smsg
import math
class SpeedCommanderTeleop:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        
        self.rate = 100 #[Hz]
        
        self.joy_val = cmsg.JointValues()

        self.enabled = False
       
        self.joy_switch_servo = []
        self.joy_switch_impedance = []
        self.arm_motor_position_ref = 0.0

        self.boom_mode = 3 
        self.arm_mode  = 3

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

    def cb_joy(self, msg):
        self.joy_val = msg
        
    def cb_state_arduino(self, msg):
        self.arm_motor_position = msg.armP 
        self.boom_motor_position = msg.boomP
                   
    def cb_joy_right(self, joy):
        self.joy_switch_servo = joy.buttons[1]
        self.joy_switch_impedance = joy.buttons[2]
        
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

            self.enabled = not self.enabled
            self.time_switch_last = rospy.get_rostime()   
    def update(self):
        r = rospy.Rate(self.rate)
        
        # Main Loop
        while not rospy.is_shutdown():
            arduino_controller_msg   = cmsg.JointCommandArduino()
            dynamixel_controller_msg = cmsg.JointCommandDynamixel()

            arduino_controller_msg.boomV   = self.joy_val.boom*100.0
            arduino_controller_msg.armV    = self.joy_val.arm*(-100.0)

            if self.arm_mode == 0:
                self.arm_motor_position_ref=self.arm_motor_position_ref + 0.1*self.joy_val.arm
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
