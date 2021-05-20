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
	while run:
		for event in pygame.event.get():
        		if event.type == pygame.QUIT:
				run = False
		keys = pygame.key.get_pressed()
      		drive=0
		steer=0
		print(keys)
		if keys[pygame.K_w]:
			drive=1
			print("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWw")
		if keys[pygame.K_s]:
			drive=-1
			print("sSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
		if keys[pygame.K_w] and keys[pygame.K_s]:
			drive=0
			print("SWSWSWSWWSWSWSWSWWSWSWSWSWS")
		if keys[pygame.K_a]:
			steer=-45
			print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
		if keys[pygame.K_d]:
			print("DDDDDDDDDDDDDDDDDDDDDDDDDdDDDDDDDDDDDDDDDDDDDDDDDDDd")
			steer=45
		if keys[pygame.K_a] and keys[pygame.K_d]:
			print("ADADADADADADDADADAADADDADADADADADADDADADAD")
			steer=0


if __name__ == '__main__':
	mouseToJoy() 
