#! /usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from vision_middleman.msg import Vision_Object
import math
import numpy as np
import time

master_slave_mode = True
prepare_for_stop_sign = False
stop = False
drive=1
steer=0

def callback_vision(msg):
    #Vision object is a custom message you can find in vision_middleman/msg/
    #string object
    #float32 distance
    #int32[] lefteye
    #int32[] righteye
    global prepare_for_stop_sign
    global stop
    vision_obj = msg.object

    if vision_obj == "stop sign":
        #distance
        distance=msg.distance
        #centroids
        centroid1=msg.lefteye
        centroid2=msg.righteye
        #do trig
        #angle_to_sign
        #we calc stop line distance
        stop_distance= distance
    
    #we monitor distance
    if stop !=True and stop_distance < 10:
        #are coming up to a stop sign and need to prep for a scenario were we lose track of the camera
        prepare_for_stop_sign=True
    if stop != True and stop_distance < 5:
        stop = True    
    #now if the stop sign goes out of view because our fov we need to just come to a stop



def main():
    global drive
    global steer
    global prepare_for_stop_sign
    global stop

    rospy.init_node('stop_sign_react')
    sub0 = rospy.Subscriber('/vision_object', Vision_Object , callback_vision)
    if master_slave_mode:
        pub_ = rospy.Publisher('/vision_drive', Float32 , queue_size=5)
        msg=Float32()
    else:
        pub_ = rospy.Publisher('joy', Joy, queue_size=5)
        msSg = Joy()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rate.sleep()
        if stop == True:
            drive=0
            if master_slave_mode:
                msg.data=drive
            else:
                #### Initialize joy msg every loop
                msg.axes = []
                msg.axes.append(steer)
                msg.axes.append(drive)
                msg.axes.append(drive)    
            pub_.publish(msg)    
            time.sleep(5)
            drive=1
            
            if master_slave_mode:
                msg.data=drive
            else:
                #### Initialize joy msg every loop
                msg.axes = []
                msg.axes.append(steer)
                msg.axes.append(drive)
                msg.axes.append(drive)    
            pub_.publish(msg)    
            time.sleep(3)
            stop=False
            prepare_for_stop_sign=False
        else:
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
