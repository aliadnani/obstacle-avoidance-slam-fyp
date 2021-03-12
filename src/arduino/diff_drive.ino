/* Arduino Pro Micro code to read ROS topic over serial and control motors
Only supports forwards and backwards movement right now */

#define USE_USBCON //  Very Important! Because Pro Micro has native USB
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <Encoder.h>
#include <PID_v1.h>

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

float wheelBase = 0.24;
float wheelDiameter = 0.08;
float wantedRight = 0;
float wantedLeft = 0;

// Target speed in rps
double Setpoint_Left;
double Setpoint_Right;

// What is read from encoder
double Input_Left;
double Input_Right;

// What you output to motor
double Output_Left;
double Output_Right;

double Kp = 40, Ki = 160, Kd = 7;
String message;

PID myPID_Left(&Input_Left, &Output_Left, &Setpoint_Left, Kp, Ki, Kd, DIRECT);
PID myPID_Right(&Input_Right, &Output_Right, &Setpoint_Right, Kp, Ki, Kd, DIRECT);

Encoder knobLeft(3, 8);
Encoder knobRight(2, 7);

ros::NodeHandle nh;

long positionLeft = -999;
long positionRight = -999;
float leftVel = 0;
float rightVel = 0;
long lastVelTime = 0;
long starttime = millis();

float convertAngularToLinear(float speed)
{
  return speed * 3.1415926 * wheelDiameter;
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

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);

  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);

  myPID_Left.SetMode(AUTOMATIC);
  myPID_Right.SetMode(AUTOMATIC);
  myPID_Left.SetTunings(Kp, Ki, Kd);
  myPID_Right.SetTunings(Kp, Ki, Kd);

  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub_vel);
}

void calculateWheelSpeed(float linear_velocity, float angular_velocity)
{
  float left_velocity = linear_velocity - (0.5f * angular_velocity * wheelBase);
  float right_velocity = linear_velocity + (0.5f * angular_velocity * wheelBase);

  wantedLeft = left_velocity;
  Setpoint_Left = fabs(left_velocity);

  wantedRight = right_velocity;
  Setpoint_Right = fabs(right_velocity);
}

void goRobot()
{
  if (wantedLeft > 0)
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
  if (wantedRight > 0)
  {
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
  }
  else
  {
    digitalWrite(In4, HIGH);
    digitalWrite(In3, LOW);
  }

  if (wantedLeft == 0.0 && wantedRight == 0.0)
  {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    digitalWrite(In3, LOW);
    digitalWrite(In4, LOW);
    analogWrite(EnA, 0);
    analogWrite(EnB, 0);
  }
  else
  {
    analogWrite(EnA, Output_Left);
    analogWrite(EnB, Output_Right);
  }
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
    calculateWheelSpeed(float(cmd_msg.linear.x), float(cmd_msg.angular.z));
  }
}

void readEncoder()
{
  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();

  if (millis() - lastVelTime > 100)
  {

    float newLeftFloat = float(newLeft);
    float newRightFloat = float(newRight);

    leftVel = fabs(((newLeftFloat / 1876.0f) / 0.1f));
    rightVel = fabs(((newRightFloat / 1876.0f) / 0.1f));

    knobLeft.write(0);
    knobRight.write(0);

    lastVelTime = millis();

    Input_Right = convertAngularToLinear(rightVel);
    Input_Left = convertAngularToLinear(leftVel);
  };
}

void PIDLoop()
{
  myPID_Right.Compute();
  myPID_Left.Compute();
}

void loop()
{
  nh.spinOnce();
  readEncoder();
  PIDLoop();
  goRobot();
}