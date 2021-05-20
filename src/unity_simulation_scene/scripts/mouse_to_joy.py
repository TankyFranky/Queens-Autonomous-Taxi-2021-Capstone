#!/usr/bin/env python
import pygame
import rospy
import numpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from Xlib import display
from Xlib.ext import randr

def mouseToJoy():
	pygame.init()
	pygame.display.set_mode((500,500))
	run = True

	# initialize node
	rospy.init_node('mouseToJoy', anonymous = True)

	#### Setup MouseToJoy Publisher 
   	mouseToJoyPublisher = rospy.Publisher("joy", Joy, queue_size = 5)
	rate = rospy.Rate(10) # 10hz
	msg = Joy()
	
	print("Is ros down?")
	print(rospy.is_shutdown())
	while not rospy.is_shutdown():
		for event in pygame.event.get():
        		if event.type == pygame.QUIT:
				pygame.quit()
		keys = pygame.key.get_pressed()
      		drive=0
		steer=0
		if keys[pygame.K_w]:
			drive=1
			print("W")
		if keys[pygame.K_s]:
			drive=-1
			print("S")
		if keys[pygame.K_w] and keys[pygame.K_s]:
			drive=0
			print("SW")
		if keys[pygame.K_a]:
			steer=-45
			print("A")
		if keys[pygame.K_d]:
			print("D")
			steer=45
		if keys[pygame.K_a] and keys[pygame.K_d]:
			print("AD")
			steer=0
		#### Initialize joy msg every loop
		msg.axes = []
		msg.buttons = []
	
		msg.axes.append(steer)		
		msg.axes.append(drive)
		msg.axes.append(drive)

		#### Publish msg
		rospy.loginfo(msg)
		mouseToJoyPublisher.publish(msg)
		rate.sleep()


if __name__ == '__main__':
	mouseToJoy()    
