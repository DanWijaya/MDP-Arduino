#include "Encoder.h"
#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"
#include "PID_v1.h"
#include <math.h>

#define enA1    3
#define enB1    5
#define enA2    11
#define enB2    13
#define wheelRadius 0.015
#define distanceBetweenWheels 0.175
#define kp 1.0
#define ki 1.0
#define kd 0.0

/**
 * Function Declarations
 */

void moveRobot(float, float);
void straight(float);
void turn(float);
void readEncoder1();
void readEncoder2();
void getRpm();
void getCmd();

DualVNH5019MotorShield md;
Encoder en(enA1, enB1, enA2, enB2);
int rpm1 = 0;
int rpm2 = 0;
unsigned long lastMilliPrint = 0;

void setup()
{
  Serial.begin(115200);
  md.init();
  en.init();

  //pull-up/down resistor
  digitalWrite(enA1, HIGH);
  digitalWrite(enA2, HIGH);
  digitalWrite(enB1, LOW);
  digitalWrite(enB2, LOW);

  //Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(enA1), readEncoder1, FALLING);
  PCintPort::attachInterrupt(enA2, readEncoder2, FALLING);
}

void loop()
{
  getCmd();
  getRpm();
}

void moveRobot(double v, double w) {
  double vl, vr;
  vl = (2 * v + w * distanceBetweenWheels)/(2 * wheelRadius);
  vr = (2 * v - w * distanceBetweenWheels)/(2 * wheelRadius);
  md.setM1Speed(-vl);
  md.setM2Speed(vr);
}

void straight(double distance) {
  double distanceL, distanceR, distanceTraversed, angular_error, v, w;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  v = 0;
  w = 0;

  PID PID_linear(&distanceTraversed, &v, &distance, kp, ki, kd, DIRECT);
  PID PID_angular(&angular_error, &w, 0, kp, ki, kd, DIRECT);
  
  while ( fabs(distanceTraversed-distance) <= 0.01 ){
    distanceL = 2 * (22/7) * wheelRadius * en.getMotor1Revs();
    distanceR = 2 * (22/7) * wheelRadius * en.getMotor2Revs();
    
    distanceTraversed = (distanceL + distanceR)/2;
    
    angular_error = distanceL - distanceR;

    PID_linear.Compute();
    PID_angular.Compute();

    moveRobot(v, w);
  }
}

void turn(double angle) {
  double distanceL, distanceR, angleTraversed, angular_error, v, w;
  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  v = 0;
  w = 0;

  PID PID_angular(&angleTraversed, &w, &angle, kp, ki, kd, DIRECT);
  
  while ( fabs(angleTraversed-angle) <= 0.001 ){
    distanceL = 2 * (22/7) * wheelRadius * en.getMotor1Revs();
    distanceR = 2 * (22/7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL)/distanceBetweenWheels;
    angleTraversed = (angleTraversed * 4068) / 71;

    PID_angular.Compute();
    
    moveRobot(0, w);
  }
}

void readEncoder1() {
  en.rencoder1();
}

void readEncoder2() {
  en.rencoder2();
}

void getCmd() {
  char cmd;
  if (!Serial.available())    return;
  delay(10);
  cmd = Serial.read();
  Serial.flush();


  /*
   * w: forward
   * s: backward
   * a: left
   * d: right
   */
  switch (cmd) {
    case 'w':
      straight(10.0);
      break;
    case 's':
      straight(-10.0);
      break;
    case 'a':
      turn(90.0);
      break;
    case 'd':
      turn(-90.0);
      break;
    default:
      break;
  }
}
