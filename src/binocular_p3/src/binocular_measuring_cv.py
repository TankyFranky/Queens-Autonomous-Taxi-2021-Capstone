#!/usr/bin/env python3

# Francesco Marrato 2021-03-17
# Hope you like lists, multithreading, and buffers

from sensor_msgs.msg import Image   # will show error but is fine
from cv_bridge import CvBridge  # will show error but is fine
from binocular_p3.msg import Vision_Object  # will show error but is fine
from colorama import Fore, Style
from scipy.spatial import distance as dist
from math import isclose
import traceback
import numpy as np
import cv2
import atexit
import sys
import time
import queue
import rospy    # will show error but is fine
import threading
import multiprocessing


#=======================================CHANGE THESE TO YOUR DIRECTORY=====================================================
#Unfortunately becuase of how the code is initialized by ros, relative paths can be pretty convoluted
#So instead we are doing absolute path
# baked YOLOv4 network files
yolo_config = "/home/francm/elec-490-2020-2021/src/binocular_p3/src/yoloV4_network_files/TinyNet/yolov4-tiny-custom-Capstone.cfg"
yolo_names = "/home/francm/elec-490-2020-2021/src/binocular_p3/src/yoloV4_network_files/obj_networkV2.names"
yolo_weights = "/home/francm/elec-490-2020-2021/src/binocular_p3/src/yoloV4_network_files/TinyNet/yolov4-tiny-custom-Capstone_last.weights"
#==========================================================================================================================

# Frame buffers for incoming camera feed
frame_buffer_left = queue.Queue(maxsize=60) # approximately 3 seconds of footage
frame_buffer_right = queue.Queue(maxsize=60)

# Frame buffers for frames pushed through the dnn
dnn_buffer_left = multiprocessing.Queue(maxsize=60)
dnn_buffer_right = multiprocessing.Queue(maxsize=60)

# CONSTANTS (created for ease of tuning, globals are not necessary in design)
KILL_THREADS = False # kills continuous multithreaded/multiprocessed tasks
CATEGORIES = None   # used for output drawing
CATEGORIES_COLORS = None
MIN_CONFIDENCE = 0.7
FRAME_HEIGHT = 576
FRAME_WIDTH = 720

# used to artificially reduce the framerate for increased blob size
left_count = 0
right_count = 0
FRAME_INTERVAL = 1

CAM_SYNC_THRESH = 0.001 * 50 #  microseconds
BLOB_SIZE = 32 * 13  # blob sizes have to be multiples of 32. Size vs Accuracy tradeoff (size up = system lag)
FOCAL_LENGTH = 35.90984# 0.03590984 # 35.90984 mm
BASELINE = 0.75 # 0.75 m


### PHYSICAL CAMERA PARAMETERS ###
""" 
Unity Physical Camera Parameters
FOV: 52.7595 H, 39.2228 V (Degrees)
Focal Length: 35.90984 (mm)
Sensor Size: 35.62mm x, 25.59mm y
Gate Fit: None
Lens Shift: 0X, 0Y
Resolution: 720x576
Baseline = 0.75m
"""
### PHYSICAL CAMERA PARAMETERS ###


def bino_instance_classifier(left_objects, right_objects):
    """
    :param left_objects: [image np.array, [[x top, y left, width, height] for each classified item]]
    :param right_objects: [image np.array, [[x top, y left, width, height] for each classified item]]
    :return: list of instances and their distances

    I'm sorry... the inputs are a bit confusing
    """

    # Left camera frame is considered reference (master) and right camera is considered target (slave)
    instances = []
    if len(left_objects[1])>0 and len(right_objects[1])>0: # if there are identified objects in both frames
        left_centroid_point = left_objects[1][:,0:2] # strip centroid position from all objects
        right_centroid_point = right_objects[1][:,0:2]
        centroid_euclidean_distances = dist.cdist(left_centroid_point, right_centroid_point, metric='euclidean')
        # ^ calculate matrix of euclidean distances from each object
        for reference_index, row in enumerate(centroid_euclidean_distances): # for each object in the left frame
            target_index = row.argmin() # find object with minimum distance to reference frame object

            # check if object width is within 15%
            size_match = isclose(left_objects[1][reference_index, 2], right_objects[1][target_index,2], rel_tol=0.15) \
                         and isclose(left_objects[1][reference_index, 3], right_objects[1][target_index,3], rel_tol=0.15)

            # Category has to match too
            category_match = True if left_objects[1][reference_index, 4] == right_objects[1][target_index,4] else False
            if size_match and category_match: # instance match found
                # Calculate Distances based off of centroid
                left_params = left_objects[1][reference_index]
                right_params = right_objects[1][target_index]
                distance = ((BASELINE*FOCAL_LENGTH)/(left_params[0]-right_params[0]))*10
                instances.append([left_params[0], left_params[1], right_params[0], right_params[1], left_params[4], round(distance,2)])

    return instances # return list of instances, including centroid position and class


def read_buffer(dnn_net, output_layer, pub, msg):
    # reads the input camera feed, passes the frames through the neural network, instance classifies, than calculates distance
    global frame_buffer_left, frame_buffer_right, dnn_buffer_left, dnn_buffer_right
    while not KILL_THREADS:

        # pull frames from camera buffers if both buffers have frames and their capture time is within the limit
        # will need shutter sync in real time system
        if frame_buffer_left.qsize()>0 and frame_buffer_right.qsize()>0:
            left_image = frame_buffer_left.get()
            right_image = frame_buffer_right.get()
            dif = abs(left_image[1].to_sec() - right_image[1].to_sec())

            if dif < CAM_SYNC_THRESH:
                # push both input frames through the dnn
                left_thread = multiprocessing.Process(name="left process", target=dnn_computation, args=(dnn_net, output_layer,left_image[0], True,))
                right_thread = multiprocessing.Process(name="right process", target=dnn_computation, args=(dnn_net, output_layer, right_image[0], False,))

                left_thread.start()
                right_thread.start()

                # ^ assure both processes are completed before continuing (process synchronization)
                if dnn_buffer_left.qsize()>0 and dnn_buffer_right.qsize()>0: # if both frames have returned classified objects
                    process_return_left = dnn_buffer_left.get()
                    process_return_right = dnn_buffer_right.get()

                    # classify instances (e.g. match the cars seen in the left eye with the car in the right eye)
                    instance = bino_instance_classifier(process_return_left, process_return_right)
                    if len(instance) > 0:

                        # TODO publish to topic
                        for d in instance:
                            msg.object =  d[4]# class
                            msg.distance = d[5] # duh

                            msg.lefteye = []  # centroid left
                            msg.lefteye =  [int(d[0]), int(d[1])]

                            msg.righteye = []  # centroid right
                            msg.righteye = [int(d[2]), int(d[3])]

                            pub.publish(msg)

                            dist_text = str(d[5])
                            cv2.putText(process_return_left[0], dist_text, (d[0], d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0) , 2)
                            cv2.putText(process_return_right[0], dist_text, (d[2], d[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 0), 2)
                    gui_output = np.concatenate((process_return_left[0],process_return_right[0]), axis=1)
                    cv2.imshow("Stereo Vision Distance Measuring - YOLOv4 - Instance Classification - Distance Measuring", gui_output)
                    cv2.waitKey(1)



            else:
                print(Fore.YELLOW + "Frame skipped, sync difference: "+ Style.RESET_ALL + str(dif) + "s")
                print("input left: " + str(frame_buffer_left.qsize()) + "dnn left: " + str(dnn_buffer_left.qsize()))
                print("input right: " + str(frame_buffer_right.qsize()) + "dnn right: " + str(dnn_buffer_right.qsize()))



def dnn_computation(yolo_net, layer_names, frame, LR):

    pre_process = cv2.dnn.blobFromImage(frame, 1 / 255.0, (BLOB_SIZE, BLOB_SIZE), swapRB=False, crop=False)
    yolo_net.setInput(pre_process)
    output_yolo = yolo_net.forward(layer_names) # Alternative could be considered (causes lag with large blob size)
    queue_frame, objects = frame_class_drawer(output_yolo, frame)
    if LR:
        dnn_buffer_left.put([queue_frame, objects],timeout=3)
    else:
        dnn_buffer_right.put([queue_frame, objects], timeout=3)


def load_dnn_yolo():
    # Loads the Darknet YOLOv4 network into OpenCV's DNN module. Preps DNN for use.
    global CATEGORIES, CATEGORIES_COLORS, FRAME_TIME_LEFT, FRAM
    CATEGORIES = open(yolo_names).read().strip().split("\n") # Pull labels from .names file
    CATEGORIES_COLORS = np.random.randint(0, 255, size=(len(CATEGORIES), 3), dtype="uint8") # assign unique color
    YOLOV4_NET = cv2.dnn.readNetFromDarknet(yolo_config, yolo_weights)
    OUTPUT_LAYER_NAMES = [YOLOV4_NET.getLayerNames()[i[0] - 1] for i in YOLOV4_NET.getUnconnectedOutLayers()]

    # retrun DNN object and Layer names of output layers
    return YOLOV4_NET, OUTPUT_LAYER_NAMES

def left_callback(msg, args):
    # called when left camera receives an image, fills the left input image buffer
    global frame_buffer_left, left_count
    if(left_count > FRAME_INTERVAL):
        left_count = 0
        left_decompressed = args[0].imgmsg_to_cv2(msg) # convert to OpenCV MAT format using cv_bridge
        try:
            buff_error = frame_buffer_left.put([left_decompressed, msg.header.stamp], timeout=3)
        except queue.Full:
            print(Fore.RED + "Left eye buffer is full" + Style.RESET_ALL)
    else:
        left_count = left_count + 1

def right_callback(msg,args):
    # same as above but for right camera
    global frame_buffer_right, right_count
    if(right_count > FRAME_INTERVAL):
        right_count = 0
        right_decompressed = args[0].imgmsg_to_cv2(msg)
        try:
            buff_error = frame_buffer_right.put([right_decompressed, msg.header.stamp], timeout=3)
        except queue.Full:
            print(Fore.RED + "Right eye buffer is full" + Style.RESET_ALL)
    else:
        right_count = right_count + 1

def frame_class_drawer(output_yolo, frame):
    global CATEGORIES, CATEGORIES_COLORS
    boxes = []
    confidences = []
    classIDs = []

    objects = []
    nms_objects = []
    for output in output_yolo:
        for instance in output:
            # extract the class ID and confidence (i.e., probability) of
            # the current object detection
            scores = instance[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]
            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
            if confidence > MIN_CONFIDENCE: # min_confidence:
                # scale the bounding box coordinates back relative to the
                # size of the image, keeping in mind that YOLO actually
                # returns the center (x, y)-coordinates of the bounding
                # box followed by the boxes' width and height
                box = instance[0:4] * np.array([FRAME_WIDTH, FRAME_HEIGHT, FRAME_WIDTH, FRAME_HEIGHT]) # TODO make W and H automatic
                (centerX, centerY, width, height) = box.astype("int")
                # use the center (x, y)-coordinates to derive the top and
                # and left corner of the bounding box
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))
                roi_limit = np.shape(frame)
                # update our list of bounding box coordinates, confidences,
                # and class IDs
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)
                classification = CATEGORIES[classID]
                objects.append(np.array([centerX, centerY, width, height, classification], dtype=object))

    idxs = cv2.dnn.NMSBoxes(boxes, confidences, MIN_CONFIDENCE, 0.3)

    if len(idxs) > 0:
        # loop over the indexes we are keeping
        for i in idxs.flatten():
            # extract the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            # draw a bounding box rectangle and label on the image
            color = [int(c) for c in CATEGORIES_COLORS[classIDs[i]]]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format(CATEGORIES[classIDs[i]], confidences[i])
            cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2)
            nms_objects.append(objects[i])

    return frame, np.asarray(nms_objects)
    # Pulled from: https://www.pyimagesearch.com/2018/11/12/yolo-object-detection-with-opencv/

def boot_print():
    print('''
    
██╗   ██╗ ██████╗ ██╗      ██████╗ 
╚██╗ ██╔╝██╔═══██╗██║     ██╔═══██╗
 ╚████╔╝ ██║   ██║██║     ██║   ██║
  ╚██╔╝  ██║   ██║██║     ██║   ██║
   ██║   ╚██████╔╝███████╗╚██████╔╝
   ╚═╝    ╚═════╝ ╚══════╝ ╚═════╝ 
                                   
██████╗ ██╗███╗   ██╗ ██████╗      
██╔══██╗██║████╗  ██║██╔═══██╗     
██████╔╝██║██╔██╗ ██║██║   ██║     
██╔══██╗██║██║╚██╗██║██║   ██║     
██████╔╝██║██║ ╚████║╚██████╔╝     
╚═════╝ ╚═╝╚═╝  ╚═══╝ ╚═════╝      
                                   
 ██████╗██╗   ██╗                  
██╔════╝██║   ██║                  
██║     ██║   ██║                  
██║     ╚██╗ ██╔╝                  
╚██████╗ ╚████╔╝                   
 ╚═════╝  ╚═══╝                    
                                   
    
    ''')


def kill_all_threads():
    # Flips KILL_THREAD boolean and holds (2s), ensuring multithreading shutdown
    global KILL_THREADS
    KILL_THREADS = True
    time.sleep(2)
    print("Killing threads...")


if __name__ == '__main__':
    try:
        boot_print()
        net, output_layer = load_dnn_yolo() # Prep and gather DNN object and output layers
        ros_bridge_cv = CvBridge() # Create cv_bridge object for converting images to MAT format
        # publisher to custom message
        bino_pub = rospy.Publisher('vision_object', Vision_Object, queue_size=5)
        bino_msg = Vision_Object()
        read_buffer_thread = threading.Thread(target=read_buffer, args=(net, output_layer, bino_pub, bino_msg)) # create thread to read incoming image buffers
        read_buffer_thread.start() # start reading from buffers
        atexit.register(kill_all_threads) # will help catch any un-killed threads
        rospy.init_node('binocular_measurement') # create node for ROS
        # subscribe to both cameras
        left_cam = rospy.Subscriber('binocular_cv/left/normal', Image, left_callback, (ros_bridge_cv, read_buffer_thread))
        right_cam = rospy.Subscriber('binocular_cv/right/normal', Image, right_callback, (ros_bridge_cv, read_buffer_thread))
        print ("try========================================================================================")
        rospy.spin()
        
    except: # kills threads in event of error
        print ("except==========================================================================================")
        traceback.print_exc()
        kill_all_threads()
    finally: # kills threads in event of proper shutdown
        print ("finally==========================================================================================")

        kill_all_threads()
