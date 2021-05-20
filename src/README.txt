
--------------------------------------------------
---- GITR'DUNN TELEOP CONTROL FROM AN ARDUINO ----
--------------------------------------------------

This package contains all the necessary components for tele-operating the GitR'Dunn vehicle via logitech joystick and Arduino Mega, as well as resetting and troubleshooting the Arduino Uno which is essential for manual operation of the vehicle. For specifics on each of the microcontrollers and their operation, see the README file in the "arduino_motor_controller" file.




-- MANUAL OPERATION --

The vehicle can be driven manually by first ensuring the steering motor is removed and the manual handlebar is attached to the steering column of the vehicle. Next, the 60A fuse switch located in the bay under the driver's seat may be flipped closed. This will provide power to both the Sabertooth 2X60 motorcontroller as well as the Arduino Uno which controls manual operation, indicated by the green LED on the dashboard of the vehicle lighting up. The vehicle key can then be turned to the "ON" position, the flashing amber light on the front warning that the vehicle is operational and may move. With the "drive method" switch on the dashboard in the "manual" position, and the "direction" switch controlling the forward and reverse motion, the vehicle may be driven by pressing the throttle pedal down and steered via the handlebars.

Note that if the throttle does not actuate the drive motor on the first try, the Uno should be reset by pressing the reset button on the shield and the throttle may be re-tested after 5 seconds. Throttle functionality should be returned once the Uno was reset and the alotted time waited.

IMPORTANT NOTE: THE UNO RUNS THROUGH A THROTTLE CALIBRATION ON STARTUP WHICH ZEROS THE THROTTLE POSITION. IT IS CRITICAL THAT THE THROTTLE PEDAL REMAIN UNTOUCHED (FULLY RELEASED) DURING STARTUP TO AVOID ANY ISSUES WHICH MAY AFFECT VEHICLE TOP SPEED.




-- TELE-OPERATION --

The vehicle can be placed in teleop mode by first installing the steer motor onto the steering coloumn of the vehicle. Then, the 60A fuse switch located in the bay of the vehicle can be flipped, providing  power to the Sabertooth motorcontroller; the green indicator LED on the dashboard will light-up when the motorcontroller is powered. The Arduino Mega which controls the teleop mode of operation can then be plugged into the computer running ROS via the USB extension built into the vehicle, and the joystick USB may be connected to the computer as well. The ROS package can then be launched using the command below. Next, the key of the vehicle can be turned to the "ON" position; the blinking amber light will then turn on indicating the vehicle is in operation and may move. With the "drive method" switch put into the "teleop" position, the vehicle can then be driven using the joystick controls which follow. 


roslaunch gitrdunn_bringup gitrdunn_teleop.launch ----- launches the bringup file which includes:joy node, twist node, arduino_commander node, and serial 
							communication node

Note that the launch file must indicate the correct port which the Arduino Mega is connected to, otherwise an error will be thrown on the ROS terminal. The default port is ttyACM0; however with 	several devices plugged in this may change. The Arduino IDE can be used to check which port the Mega is connected to via Tools->Serial Port, and the change can be reflected in the "gitrdunn_teleop.launch" file in the line: "<param name="port" value="/dev/ttyACM0"/>". It is common for this issue to arise if the Mega is unplugged from the computer while the "arduino_commander" node is running.
	
Also note that the Arduino IDE and ROS node cannot communicate with the Mega simultaneously.



-- JOY CONTROL --

Make sure the joy controller is in X position, controlled by a switch on the back side of the controller. 

deadman switch ------------------- green A button ----------------- must be held to utilize vehicle; acts as remote "kill-switch" to prevent unintentional operation of vehicle
drive motor power ---------------- right trigger ------------------ controls the throttle (power) of the drive motor proportional to amount actuated
reverse drive motor direction ---- left bumper -------------------- when held, places the vehicle in reverse mode by flipping the reversing relays
steering motor direction --------- left and right D-pad buttons --- controls the steering of the vehicle

-- SEMI AUTONOMOUS CONTROL --

roslaunch gitrdunn_bringup autonomous.launch
roslaunch taylor_navi navi.launch

change directory to taylor_navi/src
python follow_wall_real.py
python user_input.py

You should be able to use w,a,s,d keys to control the vehicle's movement
a -- turn left at next intersection
w -- continue straight
d -- turn right at next intersection
s -- stop the vehicle





-- COMMUNICATION ISSUES --
Sometimes the rosserial node will warn or throw an error stating that it has lost sync with the Arduino Mega; despite the errors and warnings the system appears to be fully operational and hence these are often ignored.




-- ROS NODES AND TOPICS --
The following outlines the various nodes and topics as they relate to this package:

/joy_node is the node which recieves the data sent by the joystick via the USB and converts it into a /joy message type, publishing it to a topic of the same name. The message includes several arrays which define the states of all the joystick controller buttons.

/teleop_twist_joy is the node which subscribes to the /joy topic and utilizes only the necessary buttons (defined in the JOY CONTROL section) to determine the desired motion. This desired motion is then published in a twist message (/cmd_vel) which includes both linear and angular x,y, and z values from 0 to 1. Note that for safety the params "scale_linear" and "scale_angular" limit the motor to approximately 30% power. This should only be increased once the reliability of the system is thoroughly tested, and even then should not be increased beyond 50% unless necessary.

/arduino_commander node subscribes to the /cmd_vel topic and maps both the drive and steer motor values to a desired power range before placing both into a 32-bit UInt message published to the /mc (which stands for microcontroller) topic.

The Arduino Mega then subscribes to the /mc topic, decodes the 32-bit message into the respective drive and steer motor power values, then communicates with the Sabertooth motorcontroller to drive the motors as desired.

The steering and drive encoders are mounted on the vehicle; however, the package which they utilize remains untested and uncalibrated.




-- PACKAGES TO DOWNLOAD --

sudo apt-get install ros-kinetic-teleop-twist-joy




