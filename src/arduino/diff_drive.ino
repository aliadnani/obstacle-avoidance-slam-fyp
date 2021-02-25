/* Arduino Pro Micro code to read ROS topic over serial and control motors
Only supports forwards and backwards movement right now */

#define USE_USBCON //  Very Important! Because Pro Micro has native USB
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

//Motor Connections
//Change this if you wish to use another diagram
#define EnA 5  // Motor A Speed PWM
#define EnB 6  // Motor A Speed PWM
#define In1 18 // A0
#define In2 19 // A1
#define In3 20 // A2
#define In4 21 // A3

double speed_req = 0;
double angular_speed_req = 0;
double speed_req_left = 0;
double speed_req_right = 0;

ros::NodeHandle nh;

void messageCb(const geometry_msgs::Twist &cmd_msg)
{

  if (cmd_msg.linear.x == 0 && cmd_msg.angular.z == 0)
  {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    digitalWrite(In3, LOW);
    digitalWrite(In4, LOW);
  }
  else
  {
    speed_req = cmd_msg.linear.x;
    angular_speed_req = cmd_msg.angular.z;

    speed_req_left = speed_req - angular_speed_req * (wheelbase / 2);
    speed_req_right = speed_req + angular_speed_req * (wheelbase / 2);

    goRobot(speed_req_left, speed_req_right);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", messageCb);
void setup()
{
  // All motor control pins are outputs
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub_vel);
}

void goRobot(double speed_left, double speed_right)
{
  // Motor A
  if (speed_right <= 0)
  {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  }
  else
  {
    digitalWrite(In2, HIGH);
    digitalWrite(In1, LOW);
  }
  analogWrite(EnA, abs(speed_left));

  // Motor B
  if (speed_left <= 0)
  {
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
  }
  else
  {
    digitalWrite(In4, HIGH);
    digitalWrite(In3, LOW);
  }
  analogWrite(EnB, abs(speed_right));
}
void loop()
{
  nh.spinOnce();
}
