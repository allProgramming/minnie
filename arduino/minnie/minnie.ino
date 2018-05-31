/*
 * rosserial client for minnie robot on Arduino
 */

#include <ros.h>
#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>  // For LSM303 I2C comms
#include <LSM303.h>
#include <PID_v1.h>

#include <minnie_serial/ToRobot.h>
#include <minnie_serial/FromRobot.h>


#define TRIGGER_PIN  11
#define ECHO_PIN     12
#define MAX_DISTANCE 450

const int TICKS_PER_REV = 40;

ros::NodeHandle  nh;


minnie_serial::FromRobot odom_msg;
ros::Publisher odom_pub("odom_pos", &odom_msg);

int LMotorSpeedPin = 5;            // Left Motor Speed pin (ENA)
int LMotorForward = A0;            // Motor-L forward (IN1)
int LMotorBackward = A1;           // Motor-L backward (IN2)

int RMotorSpeedPin = 6;            // Right Motor Speed pin (ENB)
int RMotorForward = A3;            // Motor-R forward (IN4)
int RMotorBackward = A2;           // Motor-R backward (IN3)

const int PHOTO_LEFT = 0;   //pin D2
const int PHOTO_RIGHT = 1;  //pin D3
int countL=0, countR=0;

int dirL = 0;
int dirR = 0;

Servo myservo;
int servoPos = 90;
int servoDir = 1;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

LSM303 compass;

double SetpointL,InputL,OutputL;
double SetpointR,InputR,OutputR;
PID PIDL(&InputL,&OutputL,&SetpointL,25,8,0,DIRECT);
PID PIDR(&InputR,&OutputR,&SetpointR,25,8,0,DIRECT);

void countLeft()
{
  countL += dirL;
}

void countRight()
{
  countR += dirR;
}

void driveLeft(int inLSpeed) {
  analogWrite(LMotorSpeedPin, (char)abs(inLSpeed));
  if (inLSpeed > 0) {
    digitalWrite(LMotorForward, HIGH);
    digitalWrite(LMotorBackward, LOW);
    dirL = 1;
  } else if (inLSpeed < 0) {
    digitalWrite(LMotorForward, LOW);
    digitalWrite(LMotorBackward, HIGH);
    dirL = -1;
  } else {
    dirL = 0;
  }
}

void driveRight(int inRSpeed) {
  analogWrite(RMotorSpeedPin, (char)abs(inRSpeed));
  if (inRSpeed > 0) {
    digitalWrite(RMotorForward, HIGH);
    digitalWrite(RMotorBackward, LOW);
    dirR = 1;
  } else if (inRSpeed < 0) {
    digitalWrite(RMotorForward, LOW);
    digitalWrite(RMotorBackward, HIGH);
    dirR = -1;
  } else {
    dirR = 0;
  }
}
/*
void driveMotors(int inLSpeed, int inRSpeed) {
  driveLeft(inLSpeed);
  driveRight(inRSpeed);
}
*/
int speedFromRadPerSec(float rad_per_sec) {
  if (rad_per_sec == 0) {
    return 0;
  }
  boolean isNeg = (rad_per_sec < 0);
  if (isNeg) {
    rad_per_sec = -rad_per_sec;
  }
  if (rad_per_sec < 4.0) {
    rad_per_sec = 4.0;
  }
  int s = (int)(150 / 4 * 3 + (rad_per_sec - 3.942) / (6.51 - 3.942) * (254 - 150) * 0.15);// * 1.75);
  if (s < 0) {
    s = 0;
  }
  if (s > 255) {
    s = 255;
  }
  if (isNeg) {
    s = -s;
  }
  return s;
}

void messageCb(const minnie_serial::ToRobot& msg){
  driveLeft(speedFromRadPerSec(msg.l));
  driveRight(speedFromRadPerSec(msg.r));
}

ros::Subscriber<minnie_serial::ToRobot> sub("wheel_vel", messageCb);

void setup()
{
  attachInterrupt(PHOTO_LEFT, countLeft, CHANGE);
  attachInterrupt(PHOTO_RIGHT, countRight, CHANGE);
  pinMode(LMotorSpeedPin, OUTPUT);
  pinMode(LMotorForward, OUTPUT);
  pinMode(LMotorBackward, OUTPUT);
  pinMode(RMotorSpeedPin, OUTPUT);
  pinMode(RMotorForward, OUTPUT);
  pinMode(RMotorBackward, OUTPUT);

  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-3330, -2524, -3104};
  compass.m_max = (LSM303::vector<int16_t>){+2778, +3065, +1727};

  myservo.attach(9);

  InputL = 0;
  SetpointL = 10;
  InputR = 0;
  SetpointR = 10;
  PIDL.SetMode(AUTOMATIC);
  PIDR.SetMode(AUTOMATIC);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(odom_pub);
  nh.subscribe(sub);
}

float radiansFromTicks(int count) {
  return count * 6.283185 / TICKS_PER_REV;
}

void loop()
{
  PIDL.Compute();
  PIDR.Compute();
  servoPos += servoDir;
  if (servoPos > 165 || servoPos < 15) {
    servoDir *= -1;
    servoPos += servoDir * 2;
  }
  myservo.write(servoPos);
  odom_msg.servo = myservo.read();
  int cm = sonar.ping_cm();
  odom_msg.dist = cm;
  odom_msg.l = radiansFromTicks(countL);
  odom_msg.r = radiansFromTicks(countR);
  compass.read();
  odom_msg.heading = compass.heading();
  odom_pub.publish(&odom_msg);
  nh.spinOnce();
  delay(100);
}
