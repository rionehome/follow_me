#!/usr/bin/env python
# coding: UTF-8
import rospy
import time
from std_msgs.msg import String

if __name__ == '__main__':

    rospy.init_node("follow_start_node")
    pub = rospy.Publisher('/follow_me/control', String, queue_size=10)
    time.sleep(1)
    pub.publish("start")
        
