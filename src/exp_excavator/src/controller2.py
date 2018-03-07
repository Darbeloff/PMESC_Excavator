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
        self.joint_states = smsg.JointState()
       
        self.joy_switch = []
        self.mode = 0 
        self.time_switch_last = rospy.get_rostime()
        self.initial_position_boom = 0.0
        self.sub_joy = rospy.Subscriber('joy_values', cmsg.JointValues,
                                        self.cb_joy)
        self.pub_com = rospy.Publisher('commands', cmsg.JointCommand,
                                           queue_size=10)
    
    def cb_joy(self, msg):
        self.joy_val = msg
        
       
    def cb_state(self, msg):
        self.joint_states = msg
                   
    
    def update(self):
        r = rospy.Rate(self.rate)
        
        # Main Loop
        while not rospy.is_shutdown():
            controller_msg = cmsg.JointCommand()
            controller_msg.boomV   = self.joy_val.boom*200.0
            controller_msg.armV    = self.joy_val.arm*(-200.0)
            controller_msg.bucketV = self.joy_val.bucket*2
            controller_msg.swingV  = self.joy_val.swing*0.0
          
            
            controller_msg.Kmode = self.Kmode
            self.pub_com.publish(controller_msg)
            r.sleep()
            
if __name__ == '__main__':
    sc = SpeedCommanderTeleop()
    rospy.sleep(0.2)
    try:
        sc.update()

    except rospy.ROSInterruptException:
        pass
