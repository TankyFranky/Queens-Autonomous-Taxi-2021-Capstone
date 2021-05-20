#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import math
import numpy as np


#===============================================================================================
master_slave_mode=True #If true use drive_steer_master, else run by itself, using its launch file
#===============================================================================================
#get lidar data published by read lidar, generic driving values for when m_s_mode is false
steer=0
drive=1
def callback_objects(msg):
    global drive
    drive=1
    #Components of pose to look at
    #pose.position.x = start_angle
    #pose.position.y = end_angle
    #pose.position.z = closest_range
    for pose in msg.poses:
        #print "msg data"
        #print pose.position.x
        #print pose.position.y
        #print pose.position.z
        if pose.position.x<0 and pose.position.y>0 :#then theres potential for a collision
        #slow down
            #This equation defines how it reacts look at in desmos, x is distance to object and y is drive speed (0 <-> 1)
            calc_drive = 0.43*np.log(pose.position.z-1.4)-0.2  #Approx 0 at 3m and 1 at 18m
            print calc_drive
           
            if calc_drive < drive: #set drive to lowest calculated speed
                drive=calc_drive
    

def main():
    global drive
    global steer

    rospy.init_node('obs_avoid')

    sub0 = rospy.Subscriber('/lidar_objects', PoseArray, callback_objects)
    if master_slave_mode:
        pub_ = rospy.Publisher('/lidar_drive', Float32 , queue_size=5)
        msg=Float32()
    else:
        pub_ = rospy.Publisher('joy', Joy, queue_size=5)
        msSg = Joy()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rate.sleep()
        if master_slave_mode:
            msg.data=drive
        else:
            #### Initialize joy msg every loop
            msg.axes = []
            msg.axes.append(steer)
            msg.axes.append(drive)
            msg.axes.append(drive)

        pub_.publish(msg)
if __name__ == '__main__':
    main()
