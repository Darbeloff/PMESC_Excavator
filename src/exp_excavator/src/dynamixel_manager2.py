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
        
        self.sub_spd_com = rospy.Subscriber('dynamixel_commands',
                                            cmsg.JointCommandDynamixel,
                                            self.cb_spd_com)
        self.pub_joint_bucket = rospy.Publisher('dynamixel_communication',
                                                smsg.JointState,
                                                queue_size=10)
        
    def cb_spd_com(self, msg):
        self.pub_joint_bucket.publish(name=['bucket_joint'],
                                      velocity=[msg.bucketV])

if __name__ == '__main__':
    dm = DynamixelManager()
    
    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
