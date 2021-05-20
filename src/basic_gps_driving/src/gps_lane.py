#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import matplotlib.pylab as plt
import cv2
import numpy as np
from cv_bridge import CvBridge
import math

dest_coords=None
points_to_dest=np.empty(0)
arrived=False
steer = 0
drive = 3
pt_on_list = 0
#dist_threshold = 0.000068
#dist_threshold = 0.000022888
dist_threshold = 0.00006
aggressive_steering_factor=1.4

#steering_angle = 0
steering_mode = 0
angle_to_dest = 0
gps_steer = 0
lane_steer_angle = 0

def callback_gps(msg):
    global car_coords
    car_coords = msg.axes
    #print msg.axes


def callback_gps_dest(msg):
    global points_to_dest
    global dest_coords
    points_to_dest = np.array(msg.data)
    points_to_dest = points_to_dest.reshape(-1,2)
    print points_to_dest
    dest_coords = points_to_dest[0]
    #print msg.data


def callback_heading(msg):

    global gps_steer
    global pt_on_list
    global angle_to_dest

    # Get Vehicle angle from Unity (In degrees)
    car_direction = msg.axes[0]

    # Find angle of car to destination
    #angle_to_dest = angle_of_car_to_dest(car_direction)
    angle_to_dest = angle_btw_car_and_dest(car_direction, points_to_dest)


    #VERSION 3
    if angle_to_dest > 180:
            gps_steer = -np.clip(((360-angle_to_dest)*aggressive_steering_factor),0,45)
    elif angle_to_dest <= 180:
            gps_steer = np.clip((angle_to_dest*aggressive_steering_factor),0,45)
    else:
        print 'What the hell, how did I get here?!?! ERROR'

    print 'angle2des ' , angle_to_dest , 'car d ' , car_direction , 'steer ' , gps_steer , 'car c ' , car_coords , 'dest_c' , points_to_dest[pt_on_list]
    
    # Check to see if the car has reached a waypoint
    if check_if_close_to_dest(car_coords, points_to_dest, dist_threshold):

        if pt_on_list + 1 >= len(points_to_dest):
            last_point_reached()
        else:
            print 'now going to', points_to_dest[pt_on_list]
            pt_on_list = pt_on_list + 1
            dest_coords = points_to_dest[pt_on_list]


def last_point_reached():
    print("last point reached")
    global steer
    global drive
    global arrived
    steer = 0
    drive = 3
    arrived = True


# Check to see if the car is close to the destination
def check_if_close_to_dest(car_coords, points_to_dest, dist_threshold):

    pt_1 = np.array((car_coords[0], car_coords[1]))
    pt_2 = np.array((points_to_dest[pt_on_list][0], points_to_dest[pt_on_list][1]))
    np.linalg.norm(pt_1-pt_2)

    print("dist to dest: ", np.linalg.norm(pt_1-pt_2))

    if np.linalg.norm(pt_1-pt_2) < dist_threshold:
        return True
    else:
        return False


def angle_btw_car_and_dest(car_direction, points_to_dest):
    #calculate angle between north, the car, and destination
    #opp is latitude component of car to dest, adj is longitude component
    heading=10000

    #adjacent=dest_coords[0]-car_coords[0]
    #opposite=dest_coords[1]-car_coords[1]

    adjacent = points_to_dest[pt_on_list][0] - car_coords[0]
    opposite = points_to_dest[pt_on_list][1] - car_coords[1]

    if adjacent == 0 and opposite >= 0:
        heading = 0
        return heading
    elif adjacent == 0 and opposite < 0:
        heading = 180
        return heading

    angle = np.arctan( opposite / adjacent )
    angle = np.degrees(angle)
    #We'll calculate an angle from this ranging from 0-360. Our angle calculation will result
    #in a value between 90 -> -90 since because right angle triangles do that. Adding an offset based on
    #the direction of the vector made from oppositre and adjacent would point in.
    if opposite >= 0 and  adjacent >= 0: #0-90
        heading = angle
    elif opposite > 0 and adjacent <= 0: #90-180
        heading = 180 + angle
    elif opposite < 0 and adjacent <= 0: #180-270
        heading = 180 + angle
    elif opposite < 0 and adjacent >= 0: #270-360
        heading = 360 + angle

    print 'opp: ' , opposite , 'adj; ' , adjacent, 'angle' , angle , 'car heading', car_direction , 'heading to dest',heading , (heading-car_direction)%360
    return (heading - car_direction)%360


def lane_steering(msg):
    global lane_steer_angle

    lane_steer_angle = msg.axes[0]


def main():

    global steer
    global message
    global steering_mode
    global steering_angle
    #steering_mode = 0
    #angle_to_dest = 0
    #gps_steer = 0
    #lane_steer_angle = 0

    rospy.init_node('gps_lane')

    #lane_sub = rospy.Subscriber('unity_image/normal', Image, lane_callback)
    
    # get steering angle from lane follower
    lane_angle = rospy.Subscriber('/lane_steer', Joy, lane_steering)

    # Get dest_coords from user
    sub2 = rospy.Subscriber('/dest_coords', Float32MultiArray, callback_gps_dest)

    # Don't do anything until destination coords have been imported
    while dest_coords is None:
        pass

    # Current coordinates of the vehicle
    sub0 = rospy.Subscriber('/unity_gps', Joy, callback_gps)

    # Current heading of the vehicle
    sub1 = rospy.Subscriber('/unity_heading', Joy, callback_heading)

    # The steering angle that will be sent to the vehicle
    pub = rospy.Publisher('joy', Joy, queue_size=5)

    rate = rospy.Rate(10)
    #rospy.spin()
    message = Joy()

    while not rospy.is_shutdown():
        rate.sleep()

        # if the destination is within a 140 deg cone in front of the car
        if (angle_to_dest > 40 and angle_to_dest < 320):
            steering_mode = 0
            print('gps steering!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        else:
            steering_mode = 1
            print('lane detection steering')

        # Set the steering angle based on the driving style (gps, lane)
        if steering_mode == 0:
            steering_angle = gps_steer
        elif steering_mode == 1:
            steering_angle = lane_steer_angle

        #### Initialize joy msg every loop
        message.axes = []

        print("steering angle: ", steering_angle, "gps_steer: " ,gps_steer)

        message.axes.append(steering_angle)
        message.axes.append(drive)
        message.axes.append(drive)

        #rospy.loginfo(msg)
        pub.publish(message)


if __name__ == '__main__':
    main()