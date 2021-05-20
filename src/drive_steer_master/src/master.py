#! /usr/bin/env python
import rospy
import math
import time
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Joy
angle_to_dest=None
steer=0
drive=1
steer_factors=[None,None,None,None] #{gps,lane,lidar,vision} 135 <- 135
drive_factors=[None,None,None,None] #{gps,lane,lidar,vision} -1 <-> 1

def callback_angle_to_dest(msg):
    global angle_to_dest
    angle_to_dest=msg.data

def callback_gps_steer(msg):
    global steer_factors
    #{gps,lane,lidar,vision}
    steer_factors[0]=msg.data

def callback_lane_steer(msg):
    global steer_factors
    #{gps,lane,lidar,vision}
    steer_factors[1]=msg.data

def callback_gps_drive(msg):
    global drive_factors
    #{gps,lane,lidar,vision}
    drive_factors[0]=msg.data

def callback_lidar_drive(msg):
    global drive_factors
    #{gps,lane,lidar,vision}
    drive_factors[2]=msg.data

def callback_vision_drive(msg):
    global drive_factors
    #{gps,lane,lidar,vision}
    drive_factors[3]=msg.data

def determine_drive():
    drive = 1.0
    for factor in drive_factors: 
        if factor != None and factor < drive:
            drive = factor
    return drive

def determine_steer():
    #{gps,lane,lidar,vision} 
    if angle_to_dest != None and (angle_to_dest > 40 and angle_to_dest < 320):
        #{gps,lane,lidar,vision}  
        steer=steer_factors[0]       
    else:
        steer=steer_factors[1]    

    return steer

def main():
    global drive
    global steer
    rospy.init_node('obs_avoid')
    #Subscribers
    sub0 = rospy.Subscriber('/lidar_drive', Float32 , callback_lidar_drive)
    sub1 = rospy.Subscriber('/gps_steer', Float32 , callback_gps_steer)
    sub2 = rospy.Subscriber('/gps_drive', Float32 , callback_gps_drive)
    sub3 = rospy.Subscriber('/lane_steer', Float32 , callback_lane_steer)
    sub4 = rospy.Subscriber('/gps_angle_to_dest', Float32 , callback_angle_to_dest)
    sub4 = rospy.Subscriber('/vision_drive', Float32 , callback_vision_drive)

    #Publishers
    pub_ = rospy.Publisher('joy', Joy, queue_size=5)
    msg = Joy()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #Basic choice for drive component, we pick 
        drive = determine_drive()
        steer = determine_steer()
    
        rate.sleep()
        msg.axes = []
        msg.axes.append(steer)
        msg.axes.append(drive)
        msg.axes.append(drive)
        #rospy.loginfo(msg)
        pub_.publish(msg)

if __name__ == '__main__':
    main()

