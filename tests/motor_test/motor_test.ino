#include "Encoder.h"
#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"
#include "PID_v1.h"

#include <math.h>
#include "Arduino.h"

#define enA1    3
#define enB1    5
#define enA2    11
#define enB2    13
#define wheelRadius 3
#define distanceBetweenWheels 17
#define angular_kp 70.0
#define angular_ki 61.0
#define angular_kd 1.0
/**
   Function Declarations
*/

void forward(double);
void backward(double);
void left(double);
void right(double);
void readEncoder1();
void readEncoder2();
void getRpm();
void getCmd();

/**
   Variable Declarations
*/
String instructionString = "";
String feedbackString = "";
bool stringReceived = false;

DualVNH5019MotorShield md;
Encoder en(enA1, enB1, enA2, enB2);

void setup()
{
  Serial.begin(115200);
  md.init();
  en.init();

  //  pull-up/down resistor
  digitalWrite(enA1, LOW);
  digitalWrite(enA2, LOW);
  digitalWrite(enB1, LOW);
  digitalWrite(enB2, LOW);

  //  Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(enA1), readEncoder1, FALLING);
  PCintPort::attachInterrupt(enA2, readEncoder2, FALLING);

  //  emergency brake
  //  PCintPort::attachInterrupt(A5, Export_Sensors, RISING);

  // Serial data
  instructionString.reserve(200);
}

void loop()
{
    if (stringReceived)  {
      if (instructionString == "w")
        forward(10.0);
      else if (instructionString == "s")
        backward(10.0);
      else if (instructionString == "a")
        left(90.0);
      else if (instructionString == "d")
        right (90.0);
      else
        instructionString = "";
      Serial.print("Executed " + instructionString);
      instructionString = "";
      stringReceived = false;
    }
//  delay(200);
//    forward(45.0);
//    left(90.0);
//    right(1000.0);
//    backward(50.0);
//  while (1) {}
}

void forward(double distance) {
  double distanceL, distanceR, distanceTraversed, angular_error, v, w, setPoint;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  setPoint = 0;
  v = 110;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  while ( distance - distanceTraversed > 0.5) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    //    Serial.print("   DistanceL: ");
    //    Serial.print(distanceL/100);
    //    Serial.print(" ");
    //    Serial.print("   DistanceR: ");
    //    Serial.print(distanceR/100);
    //    Serial.print(" ");
    distanceTraversed = (distanceL + distanceR) / 2;

    angular_error = distanceL - distanceR;
    Serial.print("   Forward error: ");
    Serial.println(angular_error);
    //Serial.print(" ");
    //Serial.print("   Distance left: ");
    //Serial.println(distanceTraversed);

    PID_angular.Compute();
    if (fabs(distance - distanceTraversed) < distance / 2 && v > 180) {
      v = v - 0.001 * v;
    }
    else if (v < 350) {
      v = v + 0.001 * v;
    }
    w = w - 0.001 * w;

    md.setSpeeds((-v - w) * 11 / 12, v - w);

  }
  md.setBrakes(400, 400);
}

void backward(double distance) {
  double distanceL, distanceR, distanceTraversed, angular_error, v, w, setPoint;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  setPoint = 0;
  v = 110;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  while ( fabs(distance - distanceTraversed) > 1) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    distanceTraversed = -(distanceL + distanceR) / 2;

    angular_error = distanceL - distanceR;

    //    Serial.print("   Back error: ");
    //    Serial.print(angular_error);
    //    Serial.print(" ");
    Serial.print("   Distance left: ");
    Serial.print(fabs(distance - distanceTraversed) / 100);

    PID_angular.Compute();
    if (fabs(distance - distanceTraversed) < 70 && v > 130) {
      v = v - 0.4;
    }
    else if (v < 350) {
      v = v + 1;
    }
    md.setSpeeds(v - w, -v - w);
    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.println(v / 100);
  }
  md.setBrakes(400, 400);
}

void left(double angle) {
  double distanceL, distanceR, angleTraversed, angular_error, v, w, setPoint;
  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  setPoint = 0;
  angular_error = 0;
  v = 75;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  while ( fabs(angle - fabs(angleTraversed)) > 0.1 ) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
    angleTraversed = (angleTraversed * 4068) / 71;

    angular_error = distanceL + distanceR;
    //    Serial.print("   L: ");
    //    Serial.print(distanceL);
    //    Serial.print("   R: ");
    //    Serial.print(distanceR);
    //    Serial.print("   angle left: ");
    //    Serial.print(fabs(angle - fabs(angleTraversed)) / 90);
    //    Serial.print(" ");
    Serial.print("   Left error: ");
    Serial.println(angular_error);
    PID_angular.Compute();
    if (fabs(angle - fabs(angleTraversed)) < 40.0 && v > 110) {
      v = v - 0.4;
    }
    else if (v < 250) {
      v = v + 0.4;
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.println(v / 125);
    //    Serial.print("   w: ");
    //    Serial.println(w);

    md.setSpeeds((v - w) * 11 / 12, v + w);
  }
  md.setBrakes(400, 400);
}

void right(double angle) {
  double distanceL, distanceR, angleTraversed, angular_error, v, w, setPoint;
  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  setPoint = 0;
  angular_error = 0;
  v = 75;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  while ( fabs(angle - fabs(angleTraversed)) > 0.1 ) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
    angleTraversed = (angleTraversed * 4068) / 71;

    angular_error = distanceL + distanceR;
    //    Serial.print("   L: ");
    //    Serial.print(distanceL);
    //    Serial.print("   R: ");
    //    Serial.print(distanceR);
    //    Serial.print("   angle left: ");
    //    Serial.print(fabs(angle - fabs(angleTraversed)) / 90);
    //    Serial.print(" ");
    Serial.print("   right error: ");
    Serial.println(angular_error);

    PID_angular.Compute();
    if (fabs(angle - fabs(angleTraversed)) < 40.0 && v > 110) {
      v = v - 0.4;
    }
    else if (v < 250) {
      v = v + 0.4;
    }
    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.println(v / 125);
    //    Serial.print("   w: ");
    //    Serial.println(w);
    md.setSpeeds((-v - w) * 11 / 12, w - v);
  }
  md.setBrakes(400, 400);
}

void readEncoder1() {
  en.rencoder1();
}

void readEncoder2() {
  en.rencoder2();
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    instructionString += inChar;
    if (inChar == '\n') {
      stringReceived = true;
    }
  }
}
