Two Arduino microcontrollers control the operation of the GitR'Dunn vehicle, one to control the manual operation of the vehicle and another to control the "tele-operation" (teleop) of the vehicle. The two Arduinos control the same Sabertooth 2X60 motorcontroller, as well as the same high-power relays for controlling the direction of the 4-pole, series wound DC drive motor. However, the Arduinos also have their own independent connections defined below and in the circuit/block diagram for the vehicle.


MANUAL CONTROL

	The manual control of the vehicle is done through the Arduino Uno. The connections for the Uno are as follows:
	INPUTS:		-key switch "and"ed with manual switch on dashboard; high when manual control is desired; green wire from dashboard connected to D2
			-forward switch; high when forward motion is desired and low when reverse motion is desired; blue wire from dashboard connected to D4
			-reverse switch; low when forward motion is desired and high when reverse motion is desired; blue* wire from dashboard connected to D6
			-raw throttle position; analog value ranges from 0-70 once zerod in software; orange wire from dashboard connected to A0
			-analog reference voltage; ~4.5 volts achieved using voltage divider over 12V battery which varies over time so reference voltage is necessary; orange* wire from dashboard 				connected to AREF
			-ground; connects ground of Uno to the rest of the system; black wire connected to ground rail of Uno to Sabertooth 0V terminal
			-ground; connects ground of 4-channel relay block to the rest of the system; black wire connected to ground rail of Uno to relay block GND input

	OUTPUTS:	-Tx to Sabertooth motorcontroller; sends RS232 commands which controls the drive motor via the POWER variable (0-127 range); yellow wire from Uno D11 to relay 4 NC switch
			-direction relay signal; controls the direction of the motor (high = forward, low = reverse); violet wire from D12 to relay 1 coil input
			-fuse/Sabertooth/Uno green indicator LED on dashboard; indicates whether the 60A fuse switch (and hence the Sabertooth and Uno) are powered; black wire from Uno ground rail to 			relay 3 coil input
			
	Note that for manual control, the steer motor must be disconnected from the steering column and the handlebar attached. Also note that the Uno recieves power whenever the Sabertooth is powered-on 		(controlled via the 60A fuse switch) to prevent any interferance which may cause the drive motor to unintentionally rotate. 




TELEOP CONTROL	

	The teleop control is done through the Arduino Mega in the vehicle, with connections as follows:
	INPUTS:		-key switch "and"ed with teleop switch on dashboard; high when teleop control is desired, green* wire from dashboard connected to Mega D2
			-5V used for pull-up; when Mega is off this pulls the teleop direction relay (relay 2) high so it does not interfere with manual operation; white wire on Mega connected to 				Sabertooth 5V terminal
			-ground; shares ground potential with the rest of the vehicle; black wire connected to battery 0V
		
	OUTPUTS:	-Tx to Sabertooth; sends RS232 commands to control drive and steer motors; gray wire connecting D16 to relay 4 NO
			-relay controlling Tx com with sabertooth; when pulled low by Mega switches Sabertooth com to be connected to Mega; brown wire connecting D4 on Mega to relay 4 coil input
			-drive motor reversing control via Mega; switches the high power relays that control drive motor direction using the Mega; blue wire from Mega to relay 2 coil input

	Note that the node "arduino_commander" which the Mega subscribes to publishes a 32-bit unsigned integer which holds the steer and drive motor commands. The right-most 8-bits are for the drive 	motor and the next 8-bits are for the steer motor. Each of the two parts of the message only utilize 7-bits to cover decimal range from 0-127 (where dec 0 is max rev, dec 63 is stop, and dec 127 		is max fwd). The extra empty bits are unused incase additional information must be shared from the ROS computer to the Mega.
		Ex: 0000000 = 0 = full rev..... 0111111 = 63 = stop ..... 1111111 = 127 = full fwd   


SHARED PARAMETERS
	SABERTOOTH_ADDRESS	128
	sabertooth baud rate	9600
	drive motor		1
	steer motor		2

