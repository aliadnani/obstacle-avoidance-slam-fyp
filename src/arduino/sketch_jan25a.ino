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

ros::NodeHandle nh;

void messageCb(const geometry_msgs::Twist &cmd_msg)
{

  if (cmd_msg.linear.x == 0)
  {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    digitalWrite(In3, LOW);
    digitalWrite(In4, LOW);
  }
  else
  {
    if (cmd_msg.linear.x < 0.0)
    { // derecha
      goBackward(255);
    }
    else if (cmd_msg.linear.x > 0.0)
    { // izquierda
      goForward(255);
    }
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

void goForward(int speed) //run both motors in the same direction
{
  // turn on motor A
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  // set speed to 150 out 255
  analogWrite(EnA, speed);
  // turn on motor B
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  // set speed to 150 out 255
  analogWrite(EnB, speed);
}

void goBackward(int speed) //run both motors in the same direction
{
  // turn on motor A
  digitalWrite(In2, HIGH);
  digitalWrite(In1, LOW);
  // set speed to 150 out 255
  analogWrite(EnA, speed);
  // turn on motor B
  digitalWrite(In4, HIGH);
  digitalWrite(In3, LOW);
  // set speed to 150 out 255
  analogWrite(EnB, speed);
}

void loop()
{
  nh.spinOnce();
}
