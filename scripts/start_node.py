#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import String

if __name__ == '__main__':

    while True:
        rospy.init_node("follow_start_node")
        pub = rospy.Publisher('/follow_me_nlp/follow_me', String, queue_size=10)
        pub.publish("start")
