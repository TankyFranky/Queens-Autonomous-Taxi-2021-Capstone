#! /usr/bin/env python

# LIDAR CODE
# Code finished on March 1, 2021
# Capstone Autonomous Vehicle Team

# Functinality: Scan full 270 degrees of view, find all objects within scan, save position information (start angle, end angle,range min) of object(s) detected and save information within a list. Publish list to 'lidar_objects' ROS topic

# import statements: 
import rospy
from sensor_msgs.msg import LaserScan
# from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import numpy as np
import matplotlib.pyplot as plt

# may not need, but have in case (referal to follow_wall_sim.py code):
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_msgs.msg import String
import math
import time

pub = None
sub = None

#rate = rospy.Rate(1)
#class lidar_scan:  
#    def __init__(start_angle, end_angle, closest_range): # define variables within object 
#        self.start_angle = start_angle  
#        self.end_angle = end_angle
#        self.closest_range = closest_range
 

# define callback_lidar function
def callback_lidar(msg):
    #print("enter callback_lidar") # sanity print statement
    # sanity check by outputting echo command to terminal
    #rostopic echo /unity_scan 
    range_len = 0
    range_len = len(msg.ranges)

    #print("Range length: " + str(range_len)) # sanity print statement
    #print(msg.ranges)
    
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_incr = msg.angle_increment
    range_min = msg.range_min
    range_max = msg.range_max
    time_incr = msg.time_increment

    #print( "angle_min: %0.3f" %angle_min) # sanity print statement
    #print( "angle_max: %0.3f" %angle_max) # sanity print statement
    #print( "angle_incr: %0.5f" %angle_incr) # sanity print statement
    #print( "range_min: %0.2f" %range_min) # sanity print statement
    #print( "range_max: %0.2f" %range_max) # sanity print statement

    # begin measuring lidar data
    # go through each point in the range
    i = 0
    start_angle = 0
    closet_range = 0
    end_angle = 0

    # creating list
    lidar_array = PoseArray() # create list to store lidar_scan objects 
    lidar_array.header.frame_id = 'object_positions' 

    while i < range(range_len): # loop through data point ranges within 270 degree field of view
        if msg.ranges[i] > 0: # found an object
            #print ("starting at: " + str(i)) # sanity print statement
            # print ("starting range: %0.1f" %msg.ranges[i])
           
            # this is the start of the object
            start_angle = i * angle_incr - 2.35
            closest_range = msg.ranges[i]

            # find the end of the object
            j = 1
            while msg.ranges[i+j] > 0 and (i+j) < range_len:
                end_angle = (i+j) * angle_incr - 2.35

                # capture the shortest range to car
                if msg.ranges[i+j] < closest_range:
                    closest_range = msg.ranges[i+j]
                 
                j = j + 1 

            # we are at the end of the object, reporting the data
            #print("Object between %.2f " %start_angle + " and %.2f " %end_angle + ", distance %0.1f" %closest_range) # sanity print statement
            
            # append object information to list created above
            new_pos = Pose()
            new_pos.position.x = start_angle
            new_pos.position.y = end_angle
            new_pos.position.z = closest_range

            lidar_array.poses.append(new_pos)

            # continue the scanned data in case there is another object within the range 
            i = i + j + 1

        i = i + 1
        if i >= range_len:
            break
    
    # publish list to lidar_objects topic
    pub.publish(lidar_array)

    #float32 angle_min        # start angle of the scan [rad]
    #float32 angle_max        # end angle of the scan [rad]
    #float32 angle_increment  # angular distance between measurements [rad]
    #float32 time_increment   # time between measurements [seconds] - if your scanner
		                 # is moving, this will be used in interpolating position		                 # of 3d points
    #float32 scan_time        # time between scans [seconds]
    #float32 range_min        # minimum range value [m]
    #float32 range_max        # maximum range value [m]

    #float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    #float32[] intensities    # intensity data [device-specific units].  If your
		                 # device does not provide intensities, please leave
		                 # the array empty.

#def callback_lidar_test(msg): # Test that lidar_objects topic created and data formatted properly
    #print("enter callback_lidar_test") # sanity print statement
    #print(msg) # sanity print statement to verify lists published to lidar_objects ROS topic

def main():
    global pub, sub
    #print("main") # sanity print statement
    
    # initialize range_ahead node to be used later
    rospy.init_node('lidar_scan')

    pub = rospy.Publisher('/lidar_objects', PoseArray, queue_size=10) # publish to lidar_objects topic

    # subscribe to unity_scan topic
    sub = rospy.Subscriber('/unity_scan', LaserScan, callback_lidar)

    # subscribe to lidar_objects topic (test functionality)
    #sub = rospy.Subscriber('/lidar_objects', PoseArray, callback_lidar_test)


    #rate = rospy.Rate(1)

    rospy.spin() 

# call main function
if __name__ == '__main__':
    main() # call main




