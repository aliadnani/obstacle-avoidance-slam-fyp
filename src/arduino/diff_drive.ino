/* Arduino Pro Micro code to read ROS topic over serial and control motors
Only supports forwards and backwards movement right now */

//#define USE_USBCON //  Very Important! Because Pro Micro has native USB
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <Encoder.h>
#include <PID_v1.h>

//Motor Connections
//Change this if you wish to use another diagram
#define EnA 5 // motor Left Speed PWM
#define EnB 6 // motor right Speed PWM
#define In1 8
#define In2 9
#define In3 12     // A2
#define In4 13     // A3
#define echoPin 10 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 11 //attach pin D3 Arduino to pin Trig of HC-SR04

Encoder knobRight(3, 4);
Encoder knobLeft(2, 7);

double lastEncoderLeft = 0;
double lastEncoderRight = 0;

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

double Kp = 90, Ki = 180, Kd = 10;
String message;

double raw_lin = 0;
double raw_rot = 0;

PID myPID_Left(&Input_Left, &Output_Left, &Setpoint_Left, Kp, Ki, Kd, DIRECT);
PID myPID_Right(&Input_Right, &Output_Right, &Setpoint_Right, Kp, Ki, Kd, DIRECT);

ros::NodeHandle nh;

long positionLeft = -999;
long positionRight = -999;
float leftVel = 0;
float rightVel = 0;
long lastVelTime = 0;
long starttime = millis();
long ultrasonicDistance = 0;

int MODE = 1;

long duration; // variable for the duration of sound wave travel
long distance; // variable for the distance measurement

float convertAngularToLinear(float speed)
{
  return speed * 3.1415926 * wheelDiameter;
}

//ros::NodeHandle_<ArduinoHardware, 25, 25, 360, 360> nh;
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
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);  // Sets the echoPin as an INPUT

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);

  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);

  myPID_Left.SetMode(AUTOMATIC);
  myPID_Right.SetMode(AUTOMATIC);
  myPID_Left.SetTunings(Kp, Ki, Kd);
  myPID_Right.SetTunings(Kp, Ki, Kd);

  nh.getHardware()->setBaud(57600);
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

  if (fabs(raw_lin) < 0.02 && fabs(raw_rot) < 0.02)
  {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    digitalWrite(In3, LOW);
    digitalWrite(In4, LOW);
    analogWrite(EnA, 0);
    analogWrite(EnB, 0);

    Output_Left = 0.0;
    Output_Right = 0.0;

    Setpoint_Left = 0.0;
    Setpoint_Right = 0.0;

    Input_Left = 0.0;
    Input_Right = 0.0;
  }
  else
  {
    analogWrite(EnA, Output_Left);
    analogWrite(EnB, Output_Right);
  }
}

void messageCb(const geometry_msgs::Twist &cmd_msg)
{
  if (cmd_msg.linear.x == 0.0 && cmd_msg.angular.z == 0.0)
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
  raw_lin = double(cmd_msg.linear.x);
  raw_rot = double(cmd_msg.angular.z);
}

void readEncoder()
{

  if (millis() - lastVelTime > 100)
  {
    long newLeft, newRight;

    newLeft = knobLeft.read() - lastEncoderLeft;
    newRight = knobRight.read() - lastEncoderRight;

    float newLeftFloat = float(newLeft);
    float newRightFloat = float(newRight);

    leftVel = fabs(((newLeftFloat / 1876.0f) / 0.1f));
    rightVel = fabs(((newRightFloat / 1876.0f) / 0.1f));

    // knobLeft.write(0);
    // knobRight.write(0);

    lastVelTime = millis();

    Input_Right = convertAngularToLinear(rightVel);
    Input_Left = convertAngularToLinear(leftVel);

    lastEncoderLeft = knobLeft.read();
    lastEncoderRight = knobRight.read();
  };
}

void measureUltrasonicDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
}

void PIDLoop()
{
  myPID_Right.Compute();
  myPID_Left.Compute();
}

void PIDPrep()
{
  if (fabs(raw_lin) < 0.02 && fabs(raw_rot) < 0.02)
  {
    Output_Left = 0.0;
    Output_Right = 0.0;

    Setpoint_Left = 0.0;
    Setpoint_Right = 0.0;

    Input_Left = 0.0;
    Input_Right = 0.0;
  }
}

void PIDPrepForce()
{
  Output_Left = 0.0;
  Output_Right = 0.0;

  Setpoint_Left = 0.0;
  Setpoint_Right = 0.0;

  Input_Left = 0.0;
  Input_Right = 0.0;
}

int counter = 0;

void setMode()
{
  // Read ultrasonic sensor if less than 15cm
  // set mode to 2
  measureUltrasonicDistance();
  if (MODE == 1 && distance < 25)
  {
    // if (counter < 10)
    // {
    //   counter++;
    // }

    // else
    // {

    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    digitalWrite(In3, LOW);
    digitalWrite(In4, LOW);
    analogWrite(EnA, 0);
    analogWrite(EnB, 0);

    myPID_Left.SetMode(MANUAL);
    myPID_Right.SetMode(MANUAL);

    delay(1000);
    readEncoder();

    PIDPrepForce();
    PIDLoop();
    myPID_Right.SetMode(AUTOMATIC);
    myPID_Left.SetMode(AUTOMATIC);

    MODE = 2;
  }
  // }
  // else
  // {
  //   counter = 0;
  // }
}

long clockwiseRight = 0;
long clockwiseLeft = 0;
long antiClockwiseRight = 0;
long antiClockwiseLeft = 0;

bool exitFlag = false;

void calculateSize()
{
  // bool exitFlag = false;
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);

  Setpoint_Left = 0.05;
  Setpoint_Right = 0.05;
  // knobLeft.write(0);
  // knobRight.write(0);
  readEncoder();
  PIDLoop();
  analogWrite(EnA, Output_Left);
  analogWrite(EnB, Output_Right);

  while (!exitFlag)
  {
    measureUltrasonicDistance();
    readEncoder();
    PIDLoop();
    analogWrite(EnA, Output_Left);
    analogWrite(EnB, Output_Right);

    if (distance > 40)
    {
      digitalWrite(In1, LOW);
      digitalWrite(In2, LOW);
      digitalWrite(In3, LOW);
      digitalWrite(In4, LOW);
      // clockwiseLeft = knobLeft.read();
      // clockwiseRight = knobRight.read();
      // knobLeft.write(0);
      // knobRight.write(0);
      exitFlag = true;
      myPID_Left.SetMode(MANUAL);
      myPID_Right.SetMode(MANUAL);

      delay(1000);
      readEncoder();

      PIDPrepForce();
      PIDLoop();
      myPID_Right.SetMode(AUTOMATIC);
      myPID_Left.SetMode(AUTOMATIC);
    }
  }


  exitFlag = false;
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
    while (distance > 40)
    {
      measureUltrasonicDistance();
      readEncoder();
      Setpoint_Left = 0.05;
      Setpoint_Right = 0.05;

      PIDLoop();
      analogWrite(EnA, Output_Left);
      analogWrite(EnB, Output_Right);
    }
    delay(1500);
    measureUltrasonicDistance();
    while (distance < 60)
    {
      measureUltrasonicDistance();
      readEncoder();
      Setpoint_Left = 0.05;
      Setpoint_Right = 0.05;

      PIDLoop();
      analogWrite(EnA, Output_Left);
      analogWrite(EnB, Output_Right);
    }
    while (1)
    {
      measureUltrasonicDistance();
      digitalWrite(In1, LOW);
      digitalWrite(In2, LOW);
      digitalWrite(In3, LOW);
      digitalWrite(In4, LOW);
      analogWrite(EnA, 0);
      analogWrite(EnB, 0);
    }
  // {
  //   PIDLoop();
  //   measureUltrasonicDistance();
  //   if (ultrasonicDistance > 100)
  //   {
  //     digitalWrite(In1, LOW);
  //     digitalWrite(In2, LOW);
  //     digitalWrite(In3, LOW);
  //     digitalWrite(In4, LOW);
  //     clockwiseLeft = knobLeft.read();
  //     clockwiseRight = knobRight.read();
  //     knobLeft.write(0);
  //     knobRight.write(0);
  //     exitFlag = true;
  //   }
  // }
}

void loop()
{
  setMode();
  if (MODE == 1)
  {
    nh.spinOnce();
    readEncoder();
    PIDPrep();
    PIDLoop();
    goRobot();
  }
  else
  {
    calculateSize();
  }
}