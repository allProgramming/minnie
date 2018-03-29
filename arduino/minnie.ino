/*
 * rosserial client for minnie robot on Arduino
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>


const int TICKS_PER_REV = 20;


ros::NodeHandle  nh;


std_msgs::Float32 odom_msg;
ros::Publisher odom_left_pub("odom_left_pos", &odom_msg);
ros::Publisher odom_right_pub("odom_right_pos", &odom_msg);

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
  int s = (int)(125 + (rad_per_sec - 5.544) / (47.124 - 5.544) * (250 - 125));
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

void leftMessageCb(const std_msgs::Float32& rad_per_sec){
  driveLeft(speedFromRadPerSec(rad_per_sec.data));
}

void rightMessageCb(const std_msgs::Float32& rad_per_sec){
  driveRight(speedFromRadPerSec(rad_per_sec.data));
}

ros::Subscriber<std_msgs::Float32> subL("left_wheel_vel", leftMessageCb );
ros::Subscriber<std_msgs::Float32> subR("right_wheel_vel", rightMessageCb );

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

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(odom_left_pub);
  nh.advertise(odom_right_pub);
  nh.subscribe(subL);
  nh.subscribe(subR);
}

float radiansFromTicks(int count) {
  return count * 6.283185 / TICKS_PER_REV;
}

void loop()
{
  odom_msg.data = radiansFromTicks(countL);
  odom_left_pub.publish( &odom_msg );
  odom_msg.data = radiansFromTicks(countR);
  odom_right_pub.publish( &odom_msg );
  nh.spinOnce();
  delay(100);
}
