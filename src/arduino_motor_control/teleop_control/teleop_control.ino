#include <ros.h> //ros library for communication with ros terminals
#include <std_msgs/UInt32.h> //message type used to send motor speed/direction commands to arduino
#include <Sabertooth.h> //sabertooth library which holds the ST.motor commands
#include <SoftwareSerial.h> //used for setting the motor controller serial pin to TX2  

////////////////  END OF LIBRARIES  ///////////////

#define SABERTOOTH_ADDRESS 128 //ADDRESS OF SABERTOOTH, NECESSARY FOR SERIAL COMMUNICATION
#define DIRECTION_PIN 6 //HIGH = FORWARD MOTION......LOW = REVERSE DRIVE MOTOR MOTION
#define SABERTOOTH_COM 16 //SABERTOOTH S1 (INPUT SIGNAL) CONNECTED TO TX2 (D16)
#define KEY_SWITCH 2 //CONNECTED TO DIGITAL PIN D2
#define SABERTOOTH_RELAY 4 //RELAY THAT CONTROLS WHICH TX PIN THE SABERTOOTH SEES IS CONNECTED TO THIS PIN 

SoftwareSerial SWSerial(NOT_A_PIN, SABERTOOTH_COM); // RX on no pin (unused), TX2 on pin 16 (to S1).
Sabertooth ST(SABERTOOTH_ADDRESS, SWSerial); // Address 128, and use SWSerial as the serial port.
 
ros::NodeHandle_<ArduinoHardware, 1, 0, 300, 300> nh; //1 sub, 0 pub, 300 input buffer size, 300 output buffer size

int drive_vector_new = 0;  //used for smoothing drive and steer commands to prevent jerky motion
int steer_vector_new = 0;

int drive_vector_old = 0;
int steer_vector_old = 0;

int drive_motor = 1;//driving motor connected to M1A and M1B
int steer_motor = 2;//steering motor connected to M2A and M2B

void mycb(const std_msgs::UInt32& msg) //power commands sent to motor via a 7bit (7 digit) binary which cover decimal range from 0-127; where 0 is max rev, 63 is stop, and 127 is max fwd
  {                                    //ie 0000000 = 0 = full rev........... 0111111 = 63 = stop .......... 1111111 = 127 = full fwd   
    int8_t drive_vector = (msg.data & 0xFF)-127; //defining variable motor_vector as an 8 bit integer whose values are bit wise anded(&) together with the right 7 bits(0x7F in hex) of msg.data
    int8_t steer_vector = ((msg.data & 0xFF00)>>8)-127; //shifts the mc topic input such that the 7 bits which define the steering vector are inputed as an integer to the steer_vector variable 

    
    drive_vector_new = (drive_vector_old+drive_vector)/2; //smoothing over new and old values
    steer_vector_new = (steer_vector_old+steer_vector)/2;
    
    if(drive_vector_new < 0){//REVERSE TRAVEL... NOTE THIS IS "OR"ED WITH THE MANUAL CONTROL, BUT CANNOT BE ACTUATED UNLESS THE KEY SWITCH AND TELEOP SWITCH ARE ENGAGED
      digitalWrite(DIRECTION_PIN,LOW); 
    }      
    else{//FORWARD TRAVEL ASSUMED
      digitalWrite(DIRECTION_PIN,HIGH);
    }
    
  
      if(drive_vector_new > 5 || drive_vector_new < -5){ //THIS CONDITION ENSURES THE RELAYS HAVE ENOUGH TIME TO BE FLIPPED BEFORE SENDING POWER TO MOTOR. IT IS A CRUDE METHOD AND SHOULD BE ADVANCED
        ST.motor(drive_motor,drive_vector_new); //Because motor_vector is defined as a uint8, the binary value read from msg.data is converted to decimal which is the type accepted by the ST.motor function
      }
      ST.motor(steer_motor,steer_vector_new);

    drive_vector_old = drive_vector_new;   //UPDATE OLD DRIVE AND STEERING VALUES WITH THE NEW VALUES FOR FUTURE SMOOTHING
    steer_vector_old = steer_vector_new;
  }
  
ros::Subscriber <std_msgs::UInt32> mc("mc", mycb );

void setup()
{
  delay(100); //DELAY HELPS WITH SERIAL COMMUNICATION ON ARDUINO STARTUP
  nh.initNode(); //INITIALLIZES NODE
  SWSerial.begin(9600); //SERIAL COM WITH SABERTOOTH AT THIS BAUDRATE
  ST.autobaud();
  Serial.begin(57600);//SERIAL COM WITH ROS COMPUTER/ROS MASTER
  nh.getHardware()->setBaud(57600);
  nh.subscribe(mc); //SUBSCRIBES TO THE MC (MOTOR CONTROLLER) NODE WHICH PUBLISHES A 32-BIT UINT WHICH INCLUDES THE DRIVE MOTOR AND STEERING MOTOR COMMANDS
  pinMode(KEY_SWITCH,INPUT); //KEY SWITCH "AND"ED WITH TELEOP SWITCH ON VEHICLE DASHBOARD............................................................................GREEN* WIRE CONNECTED TO TELEOP ARDUINO
  pinMode(DIRECTION_PIN,OUTPUT); //RELAY CONTROLLING THE DIRECTION OF DRIVE MOTOR; FWD = HIGH, REV = LOW; "OR"ED WITH MANUAL CONTROL
  digitalWrite(DIRECTION_PIN,HIGH);
  pinMode(SABERTOOTH_RELAY,OUTPUT); //RELAY WHICH CONTROLS WHICH SERIAL COM THE SABERTOOTH READS FROM. DEFAULT TO MANUAL ARDUINO UNLESS THIS PIN IS HIGH
  digitalWrite(SABERTOOTH_RELAY,HIGH);
}
  
void loop()
{ 
  boolean KEY_POSITION = digitalRead(KEY_SWITCH);
  if(KEY_POSITION == 1){ //ONLY SEND MOTOR COMMANDS WHEN KEYSWITCH AND TELEOP SWITCH ARE ENGAGED  
    digitalWrite(SABERTOOTH_RELAY,LOW); //SWITCHES RELAY SUCH THAT THE SABERTOOTH READS SERIAL COM FROM TELEOP ARDUINO INSTEAD OF MANUAL ARDUINO
    nh.spinOnce();
  }
  else{
    ST.motor(drive_motor,0); //ENSURE NEITHER MOTOR IS BEING ACTUATED WHEN TELEOP IS NOT DESIRED 
    ST.motor(steer_motor,0);
    digitalWrite(SABERTOOTH_RELAY,HIGH); //REMOVE SERIAL CONNECTION TO SABERTOOTH FROM TELEOP ARDUINO TO PREVENT INTERFERANCE
  }
  delay(1);
}
