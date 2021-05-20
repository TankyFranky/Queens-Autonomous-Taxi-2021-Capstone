#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32
import matplotlib.pylab as plt
import cv2
import numpy as np
from cv_bridge import CvBridge
import math


#===============================================================================================
master_slave_mode=True #If true use drive_steer_master, else run by itself, using its launch file
#=====================================================

drive = 5
steer = 0
steering_angle = 2

# Based on the left and right lane calculate the steering angle the vehicle should set
def compute_steering_angle(frame, lane_lines):

    if len(lane_lines) == 0:
        print('No lane lines detected')

        #Vehicle drifts left when steering set to 0, use 2 to compensate
        return 2

    height, width, _ = frame.shape

    # In the event of a single lane tryto keep it in an ideal position
    if len(lane_lines) == 1:
        #print 'Only detected one lane line.', lane_lines[0]
        x1, _, x2, _ = lane_lines[0][0]
        #x_offset = x2 - x1
        x_offset = x2 - 305

    # Two lanes scenario
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]

        mid = int(width / 2)
        #x_offset = ((left_x2 + right_x2) / 2 - mid) + 40
        x_offset = ((left_x2 + right_x2) / 2 - mid)

    # The y_offset sis always at half the height of the frame
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan2(x_offset , y_offset)
    steering_angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)

    return steering_angle_to_mid_deg


# Combine all the lines found in the camera feed into two seperate "lanes"
# These are the two lanes that the vehicle will attmpt to stay between
def average_slope_intercept(frame, line_segments):

    lane_lines = []

    # If there are no lines in frame return empty list
    if line_segments is None:
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    border = 1/3
    left_area_boundary = width * (1 - border)
    right_area_boundary = width * border

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print('skipping vertical line segment: %s' % line_segment)
                continue

            # Get data on line
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            
            # If the lines have the correct slope, and are on the right side of the frame
            if slope < 0:
                if x1 < left_area_boundary and x2 < left_area_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_area_boundary and x2 > right_area_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines


def make_points(frame, line):

    # Get info on the lines
    height, width, _ = frame.shape
    slope, intercept = line

    y1 = height
    y2 = int(y1 * 1 / 2)
    
    # bound the coordinates within the frame
    x1 = max(0, min(width, int((y1 - intercept) / slope)))
    x2 = max(0, min(width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


# Extract a certain region of the image
def region_of_interest(img, vertices):
    # Create a black image of same size as OG image, 
    mask = np.zeros_like(img)
    match_mask_color = 255

    # Mask the section of the image you want to keep
    cv2.fillPoly(mask, vertices, match_mask_color)

    # Mask the image and mask to only retain bottom half of the frame
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_the_lines(img, lines):
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    # In case there are no lines, return the same image without lines drawn on it
    if lines is None:
        return cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)

    for line in lines:
        for x1, y1, x2, y2 in line:

            deltaY = y2 - y1
            deltaX = x2 - x1

            cv2.line(blank_image, (x1,y1), (x2,y2), (155, 255, 0), thickness=8)

    img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
    return img

# Check the angle of the road lines, if they are not too "flat" keep them
def keep_only_road_lines(lines):

    road_lanes = []

    if lines is None:
        return lines

    # Go through each line found by the hough transform 
    for line in lines:
        for x1, y1, x2, y2 in line:

            deltaY = y2 - y1
            deltaX = x2 - x1

            # Find angle between two points
            angle = math.atan2(deltaY, deltaX)
            deg_angle = angle * (180/math.pi)

            # If the lines are too "flat", reject them as not being potential lanes
            if ((deg_angle > 10 and deg_angle < 170) or (deg_angle < -10 and deg_angle > -170)):
                road_lanes.append(line)    

    return road_lanes


def find_lanes(image):
    global steering_angle
    
    height = image.shape[0]
    width = image.shape[1]

    # Make the bottom half of the frame the region we care about
    region_of_interest_vertices = [
        (0, height),
        (0, height/2),
        (width, height/2),
        (width, height)
    ]

    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    #equalized = cv2.equalizeHist(gray_image)

    # Blur the frame as the first step
    blurred_frame = cv2.GaussianBlur(gray_image, (9,9), 0)

    # Need to experiment with other values
    canny_image = cv2.Canny(blurred_frame, 100, 120)

    # Return a cropped version of the canny image
    cropped_image = region_of_interest(canny_image,
                    np.array([region_of_interest_vertices], np.int32),)

    # Find HoughLines that will represent the road lanes
    lines = cv2.HoughLinesP(cropped_image,
                            rho=2,
                            theta=np.pi/180,
                            threshold=50,
                            lines=np.array([]),
                            minLineLength=60,
                            maxLineGap=3)


    # Show the canny image
    cv2.imshow('canny',canny_image)

    # Return the lines that are within our angle requirements
    lines_good = keep_only_road_lines(lines)

    # Draw the lines on the video
    image_with_lines = draw_the_lines(image, lines_good)

    # Show the feed with the lines drawn on
    cv2.imshow('img_with_all_lines',image_with_lines)

    # Find slope of lines on two sides of frame
    lane_lines = average_slope_intercept(image, lines_good)

    # Compute the steering angle based on the lines
    steering_angle = compute_steering_angle(image, lane_lines)

    # Send the steering angle and drive over to the vehicle/master
    #message = Joy()
    #message.axes = []

    #message.axes.append(steering_angle)
    #message.axes.append(drive)
    #message.axes.append(drive)

    #pub_.publish(message)

    return image_with_lines

# Once you receive an image frame from the vehicle, convert it into a frame usable by openCV
def lane_callback(msg):
    global current_frame

    # Create CvBridge object and convert incoming frame to cv2 format
    br = CvBridge()
    current_frame = br.imgmsg_to_cv2(msg)

    # Begin processing the frame to find lanes 
    frame = find_lanes(current_frame)

    cv2.imshow('frame', frame)

    cv2.waitKey(1)


def main():

    global pub0_
    global message

    rospy.init_node('lane_identification')

    lane_sub = rospy.Subscriber('unity_image/normal', Image, lane_callback)
    
    #pub_ = rospy.Publisher('joy', Joy, queue_size=5)

    if master_slave_mode:
        pub0_ = rospy.Publisher('lane_steer', Float32, queue_size=5)
        msg = Float32()
    else:
        pub0_ = rospy.Publisher('joy', Joy, queue_size=5)
        msg = Joy()


    rate = rospy.Rate(10)
    #rospy.spin()
    #msg = Joy()

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rate.sleep()
        if master_slave_mode:
            msg.data=steering_angle
        else:
            #### Initialize joy msg every loop
            msg.axes = []
            msg.axes.append(steering_angle)
            msg.axes.append(drive)
            msg.axes.append(drive)

        #print(steer)

        #rospy.loginfo(msg)
        pub0_.publish(msg)


if __name__ == '__main__':
    main()