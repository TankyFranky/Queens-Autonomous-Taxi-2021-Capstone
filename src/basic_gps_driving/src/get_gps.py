#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

#Input modes for how to give coords
#1=Inputing lattiude and longitude into console
#2=Use array of coordinates.
input_mode=2

#this set of coordinates should just make the car drive around in a square based on the origin
coords_input=[  44.224266052246094, -76.49555969238281,
              44.2279052734375, -76.4955825805664,
              44.22766876220703, -76.49202728271484,
              44.2248420715332, -76.49169921875,
              44.224266052246094, -76.49555969238281]

def main():
    rospy.init_node('input_gps')
    
    coords = Float32MultiArray()

    if input_mode==1:
        latitude = input("Enter latitude value: ")
        longitude = input("Enter longitude value: ")
        coords.data = [latitude , longitude]

    elif input_mode==2:
        coords.data = coords_input
    pub_ = rospy.Publisher('dest_coords', Float32MultiArray, queue_size=2)

    while pub_.get_num_connections() < 1:
        pass

    pub_.publish(coords)
    print coords



if __name__ == '__main__':
    main()