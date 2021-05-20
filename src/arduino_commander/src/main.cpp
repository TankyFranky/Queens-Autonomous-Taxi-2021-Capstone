
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
  //Topic you want to advertise
  pub = nh.advertise<std_msgs::UInt32>("mc", 1);

  //Topic you want to subscribe
  drive= nh.subscribe("drive/control_effort", 1, &SubscribeAndPublish::drive_control, this);
  steer = nh.subscribe("steering/control_effort", 1, &SubscribeAndPublish::steer_control, this);
  }


void drive_control(const std_msgs::Float64& input)
  {
    std_msgs::UInt32 output;
    int effort = int(input.data);
    int min_power = 27;
    
    if(effort> 0){
       drive_power = 127 + min_power + effort;
    }
    else if(effort < 0){
       drive_power = 127 - min_power + effort;
    }
    else{
        drive_power = 127;
    }
    
    output.data = (drive_power&0xFF) | ((steer_power&0xFF) << 8);
    pub.publish(output);
  }

void steer_control(const std_msgs::Float64& input)
  {
    std_msgs::UInt32 output;
    int effort = int(input.data);
    int min_power = 50;
    
    if(effort > 0.1){
       steer_power = 127 + min_power + effort;
    }
    else if(effort < 0.1){
       steer_power = 127 - min_power + effort;
    }
    else{
       steer_power = 127;
    }
    
    output.data = (drive_power&0xFF) | ((steer_power&0xFF) << 8);
    pub.publish(output);
  }

private:
  ros::NodeHandle nh; 
  ros::Subscriber drive;
  ros::Subscriber steer;
  ros::Publisher pub;
  int steer_power;
  int drive_power;
};//End of class SubscribeAndPublish

int main(int argc,char **argv)
{

  ros::init(argc, argv, "arduino_commander");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::Rate loop_rate(50);

  ros::spin();

  return 0;
}



