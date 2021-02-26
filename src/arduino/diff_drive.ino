/* Arduino Pro Micro code to read ROS topic over serial and control motors
Only supports forwards and backwards movement right now */

#define USE_USBCON //  Very Important! Because Pro Micro has native USB
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

//Motor Connections
//Change this if you wish to use another diagram
#define EnA 5 // motor Left Speed PWM
#define EnB 6 // motor Left Speed PWM
// Left
#define In1 18 // A0
#define In2 19 // A1
// Right
#define In3 20 // A2
#define In4 21 // A3

float WHEEL_BASE = 0.3;
float wanted_right = 0;
float wanted_left = 0;

ros::NodeHandle nh;

float mapLinearVel(float speed)
{
  float max_speed = 1;
  return (speed / max_speed) * 255.0;
}

float mapAngularVel(float speed)
{
  float max_speed = 1;
  return (speed / max_speed) * 255.0;
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

void calculateWheelSpeed(float linear_velocity, float angular_velocity)
{
  float left_velocity = linear_velocity - (0.5f * angular_velocity * WHEEL_BASE * 6.6f);
  // 1.54
  float right_velocity = linear_velocity + (0.5f * angular_velocity * WHEEL_BASE * 6.6f);
  // 0.44

  float max_vel = 1.15;
  if (left_velocity > 1.15f)
  {
    right_velocity = right_velocity / (left_velocity / 1.15f);
    left_velocity = 1.15f;
  }
  if (right_velocity > 1.15f)
  {
    left_velocity = left_velocity / (right_velocity / 1.15f);
    right_velocity = 1.15f;
  }

  wanted_left = (left_velocity / 1.15f) * 255.0f;
  wanted_right = (right_velocity / 1.15f) * 255.0f;
}

void goRobot()
{
  if (wanted_left > 0)
  {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  }
  else
  {
    digitalWrite(In2, HIGH);
    digitalWrite(In1, LOW);
  }

  // motor Right
  if (wanted_right > 0)
  {
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
  }
  else
  {
    digitalWrite(In4, HIGH);
    digitalWrite(In3, LOW);
  }

  wanted_left = fabs(wanted_left);
  analogWrite(EnA, wanted_left);
  wanted_right = fabs(wanted_right);
  analogWrite(EnB, wanted_right);
}

void goForwardLeft() { ; }
void goBackwardRight() { ; }
void goBackwardLeft() { ; }

void goBackward(int speed) //run both motors in the same direction
{
  // turn on motor Left
  digitalWrite(In2, HIGH);
  digitalWrite(In1, LOW);
  // set speed to 150 out 255
  analogWrite(EnA, speed);
  // turn on motor Right
  digitalWrite(In4, HIGH);
  digitalWrite(In3, LOW);
  // set speed to 150 out 255
  analogWrite(EnB, speed);
}

void goForward(int speed) //run both motors in the same direction
{
  // turn on motor Left
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  // set speed to 150 out 255
  analogWrite(EnA, speed);
  // turn on motor Right
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  // set speed to 150 out 255
  analogWrite(EnB, speed);
}

void turnRight(int speed) //run both motors in the same direction
{
  // turn on motor Left
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  // set speed to 150 out 255
  analogWrite(EnA, speed);
  // turn on motor Right
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  // set speed to 150 out 255
  analogWrite(EnB, speed);
}

void turnLeft(int speed) //run both motors in the same direction
{
  // turn on motor Left
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  // set speed to 150 out 255
  analogWrite(EnA, speed);
  // turn on motor Right
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  // set speed to 150 out 255
  analogWrite(EnB, speed);
}

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
    if (cmd_msg.angular.z < 0.0 && cmd_msg.linear.x > 0.0)
    {
      calculateWheelSpeed(float(cmd_msg.linear.x), float(cmd_msg.angular.z));
      goRobot();
    }
    else if (cmd_msg.angular.z > 0.0 && cmd_msg.linear.x > 0.0)
    {
      calculateWheelSpeed(float(cmd_msg.linear.x), float(cmd_msg.angular.z));
      goRobot();
    }
    else if (cmd_msg.angular.z < 0.0 && cmd_msg.linear.x < 0.0)
    {
      calculateWheelSpeed(float(cmd_msg.linear.x), float(cmd_msg.angular.z));
      goRobot();
    }
    else if (cmd_msg.angular.z > 0.0 && cmd_msg.linear.x < 0.0)
    {
      calculateWheelSpeed(float(cmd_msg.linear.x), float(cmd_msg.angular.z));
      goRobot();
    }
    else if (cmd_msg.linear.x < 0.0)
    {
      goBackward(mapLinearVel(fabs(cmd_msg.linear.x)));
    }
    else if (cmd_msg.linear.x > 0.0)
    {
      goForward(mapLinearVel(fabs(cmd_msg.linear.x)));
    }
    else if (cmd_msg.angular.z > 0.0)
    {
      turnLeft(mapAngularVel(fabs(cmd_msg.angular.z)));
    }
    else if (cmd_msg.angular.z < 0.0)
    {
      turnRight(mapAngularVel(fabs(cmd_msg.angular.z)));
    }
  }
}

void loop()
{
  nh.spinOnce();
}
