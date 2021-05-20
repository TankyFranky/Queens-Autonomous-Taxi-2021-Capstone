#!/usr/bin/env python3

import numpy as np
import cv2
import sys
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy

def lane_callback(msg):
    #print(msg)
    br = CvBridge()
    current_frame = br.imgmsg_to_cv2(msg)
    cv2.imshow("test", current_frame)

def main():
    print("#################### I've started and I'm running: " + sys.version)
    rospy.init_node('lane_identification')
    lane_sub = rospy.Subscriber('unity_image/normal', Image, lane_callback)
    rospy.spin()


if __name__ == '__main__':
    main()