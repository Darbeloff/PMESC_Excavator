#!/usr/bin/python

'''
Created on Feb 24, 2017

@author: yutak
'''

import rospy
import sensor_msgs.msg as smsg
import exp_excavator.msg as cmsg

class DynamixelManager:
    def __init__(self):
        rospy.init_node('dynamixel_manager', anonymous=True)
        
        self.sub_spd_com = rospy.Subscriber('spd_commands',
                                            cmsg.JointValues,
                                            self.cb_spd_com)
        self.pub_joint_bucket = rospy.Publisher('joint_commands',
                                                smsg.JointState,
                                                queue_size=10)
        
    def cb_spd_com(self, msg):
        self.pub_joint_bucket.publish(name=['bucket_joint'],
                                      velocity=[msg.bucket])

if __name__ == '__main__':
    dm = DynamixelManager()
    
    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass