#! /usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from std_msgs.msg import String

import sys, select, termios, tty

def talker():
	pub = rospy.Publisher('user_input', String, queue_size=1)
	rospy.init_node('talker')
	rate = rospy.Rate(20)
	#command = raw_input("Input F,W,R")
	
	while not rospy.is_shutdown():
		str = raw_input("w,a,s,d for direction control:  ")
		rospy.loginfo(str)
		pub.publish(str)
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass


