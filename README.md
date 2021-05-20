# Taylor Dunn Autonomous Campus Taxi | 2020-2021 ELEC 490 Capstone Team 17
## Project Description
The scope and development of the project was changed significantly due to the restrictions caused by the Covid-19 pandemic. Team 17 has developed an automated driving system for the Taylor-Dunn 534 autonomous vehicle with a focus on environment simulation and software.  The goal is to accelerate software development to allow vehicle integration and hardware testing at an earlier stage when received by the succeeding team. Without access to hardware, the project was redesigned to include the development of a robust and accurate 1:1 copy of a large portion of Queen's Campus. The virtual environment was used for testing of the projects other major components, including computer vision, GPS guidance, LiDAR object detection, and path planning.


![](https://media.giphy.com/media/H7rpSYHRyYgamxQNqw/giphy.gif)

The direction of this legacy project was changed significantly in 2020 to transition the Taylor Dunn Autonomous vehicle for outdoor urban environments. Previously, the vehicle was designed as a indoor delivery/service robot and to map unfamilliar indoor spaces. The approach for 2020-2021 was to use the existing drive-by-wire control system and integrate it with new computer vision and path planning techniques so that the autonomous vehicle could be used in (relatively) small outdoor urban environments. The project goal was to develop a system to help thos with physical limitations traverse between the main lecture halls on Queen's campus, eliminating the need for the school to hire private taxi services.
## Project Components
#### Unity Virtual Environment
The 1:1 virtual recreation of Queen's campus was developed to be used as the simulation testing environment in response to the Covid-19 pandemic. Without access to campus, a large scale, easily manipulated, virtual testing grounds was needed to complete project validation. The simulated environment runs in the Unity game development software through integration with Siemens ROS# connection. We ahieved an accurate scale and layout by using the Google Maps Unity API and a 1:1 LiDAR scan of the kingston area provided by the City of Kingston. Roadways and sidewalks were prioritized for accuracy and developed using the EasyRoads3D package for Unity. This package creates a variety of road types with an easy to use GUI. The final environment included street lights, signal lights, stop signs, and driveable RC cars to tests interactions with the autonomous vehicle.

The Unity virtual environment is not available in this repository but can be seen at the following link (limited to Queen's students only):

![](https://i.imgur.com/SPv84xw.png)


[1:1 Queen's Virtual Environment (Unity)](https://code.engineering.queensu.ca/elec490-autonomous-driving/elec-490-2020-2021-unity)

#### Lane Following Computer Vision
Using a centered camera on the simulated vehicle and OpenCV, we are able to keep the vehicle centered in the lane while traveling between GPS coordinates. Gaussian blur, Canny edge detection, and Hough transforms are used to identify the roadlines and lane boundaries. Using the lane boundaries, lane line projections are created and steering adjustments are made to stay centered between the two lines. 

![](https://i.imgur.com/6bNpZUK.png)


#### Stereo Distance Measuring Computer Vision
A stereo vision (two camera) system that is capable at identifying key environment objects (cars, trucks, motocycles, buses, stop signs, traffic lights) and estimating a distance (in meters) to the target. The system identifies objects in the environment using a YOLOv4 deep neural network trained using the Darknet framework. Object detection is completed by passing incoming video frames through the OpenCV deep neural network methods/object. After objects have been identified, instance classification is completed by comparing what is seen in the left camera vs. the right camera image frames. For an object instance to be classified, objects must exist in both the left and right frames with matching classes, and widths/heights within 13% of each other. Objects are tracked using euclidean distance. The distance estimation is completed using epipolar geometry. Since virtual cameras are used, intrinsic and extrinsic camera parameters and constant and calibration is un-necessary.


![](https://i.imgur.com/p2xF7GE.gif)

#### GPS Guidance
A GPS guidance control loop which references a list of longitude/latitude pairs corresponding to the intersections between vehicle starting point and destination. The control loop compares the current orientation of the front wheel of the vehicle and the desired direction of travel to the next set of coodinate pairs. Knowing the difference in position the vehicle can make steering adjustments as it continues to move forward. Smoothing and overshoot prevention are handled using PID.

![](https://i.imgur.com/Hx8d40v.png)

#### LiDAR Object Detection
A simulated LiDAR sensor is included on the virtual vehicle. The virtual LiDAR is tuned to mimic the capabilities of the SICK LMS5XX single band 2D lidar, returning a 270 degree single beam view of the front and sides of the vehicle. The LiDAR control system scans the 270 FOV and returns objects within a 20 meter range and their approximate position (in degrees) relative to the vehicles heading. The vehicle will react by slowing down when an object is detected by the LiDAR to be in its path. The vehicle uses an speed ramp/curve to gradually come to a stop before comming into contact with any objects.

![](https://i.imgur.com/qao1mtb.png)

#### Path Planning
A non-integrated component of the project. Uses components of the Open Street Maps API to gather and store GPS locations associated with key intersections across Queen's campus. Given a starting and ending location, will output an ordered list of GPS coordinates that can be followed to arrive at the destination. Was planned to be combined with the GPS Guidance component of the project but was left isolated because of time constraints.

![](https://i.imgur.com/P89b3Gq.png)

## Git Folder Breakdown
Below is a reduced list of the folders that should be reviewed to understand the major functions of the project. Many of the folders have descript names and all code in these folders should be fully commented. Maybe later we will add descriptions and what is included in each folder.

##### basic_gps_driving 
##### binocular_p3
##### drive_steer_master
##### file_server
##### lidar
##### m2wr_description
##### simulation
### Group Members | Contact Info | Project Focus
Below is a table containing the project group members, their contact info, and the components of the project that they are most knowledgable about. If you are having problems understanding project components or would like a better explanation of how the systems above were developed, please feel free to reach out to us.

|Name|Contact|Focus/Contribution|
|-------------------|----------------------------------------------|--------------------------------------------------|
|Ayrton Foster|ayrtonfoster@gmail.com|Lane Following CV + GPS integration|
| Eric Leask        | eric.f.leask@gmail.com                       | GPS Guidance + Unity Virtual Environment         |
| Francesco Marrato | unkindthrower@gmail.com or 15fram@queensu.ca | Stereo CV + YOLO DNN + Unity Virtual Environment |
| Patrick Watson    |                                              | Lidar Object Detection + Path Planning           |



