#!/usr/bin/env python
# coding: UTF-8

import rospy
from sound_system.srv import HotwordService
from std_msgs.msg import String


class FollowMeHotWord:
	def __init__(self):
		rospy.init_node("follow_me_hot_word")

		self.status = True
		self.control_pub = rospy.Publisher("/follow_me/control", String, queue_size=10)

		self.main()

	def main(self):
		while not rospy.is_shutdown():
			self.hot_word()
			self.status = not self.status
			if self.status:
				self.control_pub.publish("start")
			else:
				self.control_pub.publish("stop")

	def hot_word(self):
		"""
		「hey, ducker」に反応
		:return:
		"""
		rospy.wait_for_service("/sound_system/hotword", timeout=1)
		print "hot_word待機"
		rospy.ServiceProxy("/sound_system/hotword", HotwordService)()


if __name__ == '__main__':
	FollowMeHotWord()
	rospy.spin()
