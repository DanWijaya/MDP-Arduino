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
#define kp 100.0
#define ki 0.0
#define kd 0.0
#define angular_kp 70.0
#define angular_ki 65.0
#define angular_kd 1.1

#define LONG 20150
#define SHORT 1080

#define irFM A0
#define irFL A1
#define irFR A2
#define irLF A3
#define irLM A4
#define irRM A5

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

SharpIR sensorFR(irFR, 1, 93, SHORT);  // (pin , no.reading b4 calculate mean dist, diff btw 2 consecutive measu taken as valid)
SharpIR sensorFL(irFL, 1, 93, SHORT);
SharpIR sensorFM(irFM, 1, 93, SHORT);
SharpIR sensorLM(irLM, 1, 93, SHORT);
SharpIR sensorLF(irLF, 1, 93, SHORT);
SharpIR sensorRM(irRM, 1, 93, LONG);

void setup()
{
  Serial.begin(9600);
  md.init();
  en.init();

  pinMode(irFR, INPUT);
  pinMode(irFL, INPUT);
  pinMode(irFM, INPUT);
  pinMode(irRM, INPUT);
  pinMode(irLF, INPUT);
  pinMode(irLM, INPUT);

  //  pull-up/down resistor
  digitalWrite(enA1, LOW);
  digitalWrite(enA2, LOW);
  digitalWrite(enB1, LOW);
  digitalWrite(enB2, LOW);
  digitalWrite(irFR, LOW);
  digitalWrite(irFL, LOW);
  digitalWrite(irFM, LOW);
  digitalWrite(irRM, LOW);
  digitalWrite(irLM, LOW);
  digitalWrite(irLF, LOW);

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
  Export_Sensors();
}

void forward(double distance) {
  double distanceL, distanceR, distanceTraversed, angular_error, v, w, setPoint;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  setPoint = 0;
  v = 75;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  while ( fabs(distance - distanceTraversed) > 1) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    distanceTraversed = (distanceL + distanceR) / 2;

    angular_error = distanceL - distanceR;
    PID_angular.Compute();
    if (fabs(distance - distanceTraversed) < 70 && v > 130) {
      v = v - 0.4;
    }
    else if (v < 350) {
      v = v + 1;
    }

    md.setSpeeds(-v - w, v - w);

  }
  md.setSpeeds(0, 0);
  Serial.println("Forward:10");
}

void backward(double distance) {
  double distanceL, distanceR, distanceTraversed, angular_error, v, w, setPoint;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  setPoint = 0;
  v = 75;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  while ( fabs(distance - distanceTraversed) > 1) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    distanceTraversed = -(distanceL + distanceR) / 2;

    angular_error = distanceL - distanceR;
    PID_angular.Compute();
    if (fabs(distance - distanceTraversed) < 70 && v > 130) {
      v = v - 0.4;
    }
    else if (v < 350) {
      v = v + 1;
    }
    md.setSpeeds(v - w, -v - w);

  }
  md.setSpeeds(0, 0);
  Serial.println("Backward:10");
}

void left(double angle) {
  double distanceL, distanceR, angleTraversed, angular_error, v, w, setPoint;
  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  setPoint = 0;
  angular_error = 0;
  v = 80;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  while ( fabs(angle - fabs(angleTraversed)) > 0.8 ) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
    angleTraversed = (angleTraversed * 4068) / 71;

    angular_error = distanceL + distanceR;

    PID_angular.Compute();
    if (fabs(angle - fabs(angleTraversed)) < 80.0 && v > 100) {
      v = v - 0.4;
    }
    else if (v < 250) {
      v = v + 0.4;
    }
    md.setSpeeds(v - w, v + w);
  }
  md.setSpeeds(0, 0);
  Serial.println("Left:90");
}

void right(double angle) {
  double distanceL, distanceR, angleTraversed, angular_error, v, w, setPoint;
  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  setPoint = 0;
  angular_error = 0;
  v = 60;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  while ( fabs(angle - fabs(angleTraversed)) > 0.9 ) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
    angleTraversed = (angleTraversed * 4068) / 71;

    angular_error = distanceL + distanceR;

    PID_angular.Compute();
    if (fabs(angle - fabs(angleTraversed)) < 45.0 && v > 100) {
      v = v - 0.4;
    }
    else if (v < 250) {
      v = v + 0.4;
    }
    md.setSpeeds(-v - w, w - v);
  }
  md.setSpeeds(0, 0);
  Serial.println("Right:90");
}

void readEncoder1() {
  en.rencoder1();
}

void readEncoder2() {
  en.rencoder2();
}

void Export_Sensors() {
  String resultFR  = String("fr:") + String(final_MedianRead(irFR));
  String resultFL  =  String("fl:") + String(final_MedianRead(irFL));
  String resultFM  =  String("fm:") + String(final_MedianRead(irFM));
  String resultLF = String("lf:") + String(final_MedianRead(irLF));
  String resultLM = String("lm:") + String(final_MedianRead(irLM));
  String resultRM = String("rm:") + String(final_MedianRead(irRM));
  Serial.println(resultFR + resultFL + resultFM + resultLF + resultLM + resultRM); 

}

double final_MedianRead(int tpin) {
  double x[21];

  for (int i = 0; i < 21; i ++) {
    x[i] = distanceFinder(tpin);
  }
  insertionsort(x, 21);
  return x[10];
}


int distanceFinder(int pin)
{
  int dis = 0;
  switch (pin)
  {
    case irFR:
      dis = sensorFR.distance();
      break;
    case irFL:
      dis = sensorFL.distance();
      break;
    case irFM:
      dis = sensorFM.distance();
      break;
    case irLF:
      dis = sensorLF.distance();
      break;
    case irLM:
      dis = sensorLM.distance();
      break;
    case irRM:
      dis = sensorRM.distance();
      break;
    default:
      break;
  }
  return dis;
}

void insertionsort(double array[], int length)
{
  double temp;
  for (int i = 1; i < length; i++) {
    for (int j = i; j > 0; j--) {
      if (array[j] < array[j - 1])
      {
        temp = array[j];
        array[j] = array[j - 1];
        array[j - 1] = temp;
      }
      else
        break;
    }
  }
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
