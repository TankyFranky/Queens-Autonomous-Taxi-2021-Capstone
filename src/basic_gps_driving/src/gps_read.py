#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Float32
import math
import numpy as np

#===============================================================================================
master_slave_mode=True #If true use drive_steer_master, else run by itself, using its launch file
#=====================================================

dest_coords=None
car_coords = None
points_to_dest=np.empty(0)
arrived=False
steer = 0
drive = 1
pt_on_list = 0
dist_threshold = 0.00006
aggressive_steering_factor=1.4 #The 
angle_to_dest= 0.0

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

    global steer
    global pt_on_list
    global angle_to_dest
    # Get Vehicle angle from Unity (In degrees)
    car_direction = msg.axes[0]

    ############ UNCOMMENT THIS TO GET WAYPOINT TOWARDS DEST ##########
    #points_to_dest = get_points_on_line()
    
    # or
    #points_to_dest = [dest_coords]

    # Find angle of car to destination
    #angle_to_dest = angle_of_car_to_dest(car_direction)
    angle_to_dest = angle_btw_car_and_dest(car_direction, points_to_dest)

    # Get logic to decide how much car needs to turn
    # A Crude first version, the car will attemp to steer towards the point
    # Aggressivly at first, then gradually lessen the turn angle as it
    # gets closer to pointing in the right direction
    # The vehicle will probably continously make micro adjustments

    # VERSION 2
    
    '''if angle_to_dest > 45 and angle_to_dest < 180:
        steer = 45
    elif angle_to_dest > 45 and angle_to_dest > 180:
        steer = -45

    if angle_to_dest >= 0 and angle_to_dest <= 45 and angle_to_dest < 180:
        steer = angle_to_dest
    elif angle_to_dest >= 0 and angle_to_dest <= 45 and angle_to_dest > 180:
        steer = -angle_to_dest
    print 'angle2d ' , angle_to_dest , 'car d ' , car_direction , 'steer ' , steer , 'car c ' , car_coords , 'dest_c' , points_to_dest[pt_on_list]


    if (abs(abs(car_coords[0]) - abs(points_to_dest[pt_on_list][0]))) < 0.5 and abs((abs(car_coords[1]) - abs(points_to_dest[pt_on_list][1]))) < 0.5:
        print(abs(car_coords[0]) - abs(points_to_dest[pt_on_list][0]))
        print(abs(car_coords[1]) - abs(points_to_dest[pt_on_list][1]))
        pt_on_list = pt_on_list + 1
        print 'now going to', points_to_dest[pt_on_list]'''


    #VERSION 3
    if angle_to_dest > 180:
        #if (360-angle_to_dest) > 45:
        #   steer = -45
        #else:
            steer = -np.clip(((360-angle_to_dest)*aggressive_steering_factor),0,45)
    elif angle_to_dest <= 180:
        #if (angle_to_dest) > 45:
        #   steer = 45
        #else:
            steer = np.clip((angle_to_dest*aggressive_steering_factor),0,45)
    else:
        print 'What the hell, how did I get here?!?! ERROR'

    print 'angle2d ' , angle_to_dest , 'car d ' , car_direction , 'steer ' , steer , 'car c ' , car_coords , 'dest_c' , points_to_dest[pt_on_list]
    
    # Check to see if the car has reached a waypoint
    if check_if_close_to_dest(car_coords, points_to_dest, dist_threshold):

        
        if pt_on_list + 1 >= len(points_to_dest):
            last_point_reached()
        else:
            print 'now going to', points_to_dest[pt_on_list]
            pt_on_list = pt_on_list + 1
            dest_coords = points_to_dest[pt_on_list]

def last_point_reached():
    global steer
    global drive
    global arrived
    steer = 0
    drive = 0
    arrived = True

# Check to see if the car is close to the destination
def check_if_close_to_dest(car_coords, points_to_dest, dist_threshold):
    '''Both_true = 0

    if (abs(abs(car_coords[0]) - abs(points_to_dest[pt_on_list][0]))) < dist_threshold:
        Both_true += 1

    if (abs(abs(car_coords[1]) - abs(points_to_dest[pt_on_list][1]))) < dist_threshold:
        Both_true += 1

    if Both_true == 2:
        return True'''

    pt_1 = np.array((car_coords[0], car_coords[1]))
    pt_2 = np.array((points_to_dest[pt_on_list][0], points_to_dest[pt_on_list][1]))
    np.linalg.norm(pt_1-pt_2)

    if np.linalg.norm(pt_1-pt_2) < dist_threshold:
        return True
    else:
        return False


# Get a set of points along the line from the start to the destination
def get_points_on_line():
    start_coords = [10,10]
    #dest_coords = [20,10]
    interval = 4

    # For now set the start point to a static value
    points_on_line = [start_coords]

    # Calculate the vector from the start to the destination 
    a = dest_coords[1] - start_coords[1]
    b = dest_coords[0] - start_coords[0]

    # Find the slop to the destination /////NOT USED RIGHT NOW///
    slope = (dest_coords[1]-start_coords[1])/(dest_coords[0]-start_coords[0])

    # Create a vector 
    line_vector = np.array([b,a])

    # Create normalized vector from start to dest
    line_unit_vector = line_vector / np.linalg.norm(line_vector)
        
    # tavel from the start to the dest, create a point at a set interval along the line
    # Stop when the next point would go beyond the detination, only works for positive destinations right now
    while points_on_line[-1][0] + (interval * line_unit_vector[0]) < abs(dest_coords[0]) and points_on_line[-1][1] + (interval * line_unit_vector[1]) < abs(dest_coords[1]):
        x_line_pt = points_on_line[-1][0] + (interval * line_unit_vector[0])
        y_line_pt = points_on_line[-1][1] + (interval * line_unit_vector[1])

        points_on_line.append([x_line_pt, y_line_pt])


    points_on_line.append(dest_coords)
    return points_on_line

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

def angle_of_car_to_dest(car_direction):
    #convert from degrees to radians
    car_dir_rad = ((car_direction-90)*(-1)) * (math.pi/180)

    # Convert to x and y direction
    car_x_dir = math.cos(car_dir_rad)
    car_y_dir = math.sin(car_dir_rad)

    # Convert to numpy arrays
    vector_1 = np.array(dest_coords)
    vector_2 = np.array([car_x_dir , car_y_dir])

    # Find unit vectors for each point
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)

    # Get dot product
    dot_product = np.dot(unit_vector_1, unit_vector_2)

    # Find angle between two points
    angle = np.arccos(dot_product)
    deg_angle = angle * (180/math.pi)

    #print(deg_angle)
    return deg_angle

def main():
    print "Howdy partner"
    rospy.init_node('read_gps')

    global dest_coords

    sub2 = rospy.Subscriber('/dest_coords', Float32MultiArray, callback_gps_dest)

    if master_slave_mode:
        pub0_ = rospy.Publisher('/gps_steer', Float32, queue_size=5)
        pub1_ = rospy.Publisher('/gps_drive', Float32, queue_size=5)
        pub2_ = rospy.Publisher('/gps_angle_to_dest', Float32, queue_size=5)
        steer_msg=Float32()
        drive_msg=Float32()
        angle_to_dest_msg=Float32()
    else:
        pub0_ = rospy.Publisher('joy', Joy, queue_size=5)
        msg = Joy()

    while dest_coords is None:
        if master_slave_mode:
            drive_msg.data=0
            pub1_.publish(drive_msg)
        else:
            msg.axes = []
            msg.axes.append(0)
            msg.axes.append(0)
            msg.axes.append(0)
            pub0_.publish(msg)


    sub0 = rospy.Subscriber('/unity_gps', Joy, callback_gps)
    sub1 = rospy.Subscriber('/unity_heading', Joy, callback_heading)

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown() and not arrived:
        rate.sleep()
        if master_slave_mode:
            steer_msg.data=steer
            angle_to_dest_msg.data=angle_to_dest
            drive_msg.data=drive
            pub0_.publish(steer_msg)
            pub1_.publish(drive_msg)
            pub2_.publish(angle_to_dest_msg)
        else:
            #### Initialize joy msg every loop
            msg.axes = []
            msg.axes.append(steer)
            msg.axes.append(drive)
            msg.axes.append(drive)
            pub0_.publish(msg)
            
    print "destination arrived"
    if master_slave_mode:
        drive_msg.data=0
        pub1_.publish(drive_msg)
if __name__ == '__main__':
    main()
