
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
  //Topic you want to advertise
  pub = nh.advertise<std_msgs::UInt32>("mc", 1);

  //Topic you want to subscribe
  sub = nh.subscribe("/cmd_vel", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const geometry_msgs::Twist& input)
  {
    std_msgs::UInt8 drive_vector;
    std_msgs::UInt8 steer_vector;
    std_msgs::UInt32 output;

    float deadband = 0.03;	  

    if(input.linear.x < 0.00-deadband){  //reverse drive motor	
      drive_vector.data = ((input.linear.x)*127+127);
    }
    else if(input.linear.x > 0.00+deadband){	//forward drive motor
      drive_vector.data = ((input.linear.x)*127+127);
    }
    else{
      drive_vector.data = 127; //127 is zero motion for drive motor
    }

    if(input.angular.z > deadband | input.angular.z < deadband){	
      steer_vector.data = ((input.angular.z)*127+127);
    }
    else{
      steer_vector.data = 127; //127 is zero motion for drive motor
    }
    output.data = (drive_vector.data&0xFF) | ((steer_vector.data&0xFF) << 8);
    pub.publish(output);
  }

private:
  ros::NodeHandle nh; 
  ros::Publisher pub;
  ros::Subscriber sub;

};//End of class SubscribeAndPublish

int main(int argc,char **argv)
{

  ros::init(argc, argv, "arduino_commander");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::Rate loop_rate(20);

  ros::spin();

  return 0;
}



