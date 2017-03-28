#include "Encoder.h"
#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"
#include "PID_v1.h"

#include "math.h"
#include "Arduino.h"

#define enA1    11
#define enB1    13
#define enA2    3
#define enB2    5
#define wheelRadius 3.1
#define distanceBetweenWheels 17

/*
   PID Values
*/
#define angular_kp 65.0
#define angular_ki 65.0
#define angular_kd 1.0

#define turn_kp 52.0
#define turn_ki 50.0
#define turn_kd 1.0

/*
   Sensor Settings
*/

#define FL 1
#define FR 2
#define LF 3
#define LM 4
#define RM 5
#define FM 0

#define irFM A0
#define irFL A4
#define irFR A5
#define irLF A1
#define irLM A3
#define irRM A2
#define distBtwnFLR 17

/**
   Variable Declarations
*/

String instructionString = "";
String feedbackString = "";
String stringValueReceived = "";
int intValueReceived = 0;
bool stringReceived = false;
long timeStartAlign = 0;

DualVNH5019MotorShield md;
Encoder en(enA1, enB1, enA2, enB2);

SharpIR sensorFR(irFR, 10, 93, FR);  // (pin , no.reading b4 calculate mean dist, diff btw 2 consecutive measu taken as valid)
SharpIR sensorFL(irFL, 10, 93, FL);
SharpIR sensorLM(irLM, 10, 93, LM);
SharpIR sensorLF(irLF, 10, 93, LF);
SharpIR sensorRM(irRM, 10, 93, RM);
SharpIR sensorFM(irFM, 10, 93, FM);


void setup()
{
  Serial.begin(115200);
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
  digitalWrite(irRM, LOW);
  digitalWrite(irLM, LOW);
  digitalWrite(irLF, LOW);
  digitalWrite(irFM, LOW);


  //  Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(enA2), readEncoder2, FALLING);
  PCintPort::attachInterrupt(enA1, readEncoder1, FALLING);

  PWM_Mode_Setup();

  // Serial data
  instructionString.reserve(200);
}

void loop()
{
  if (stringReceived)  {
    if (instructionString[0] == 'm') {
      if (instructionString[1] == 'w') {
        forward(intValueReceived, false);
        distAlign();
        sendInfo(instructionString[1]);
      }
      else if (instructionString[1] == 's') {
        backward(intValueReceived);
        sendInfo(instructionString[1]);
      }
      else if (instructionString[1] == 'a') {
        left(90.0, false);
        distAlign();
        sendInfo(instructionString[1]);
      }
      else if (instructionString[1] == 'd') {
        right (90.0, false);
        distAlign();
        sendInfo(instructionString[1]);
      }
      else if (instructionString[1] == 'l') {
        timeStartAlign = millis();
        wall_alignment();
        sendInfo(instructionString[1]);
      }
      else
        instructionString = "";
    }
    else if (instructionString.equals(String("start\n"))) {
      sendInfo('e');
    }
    instructionString = "";
    stringValueReceived = "";
    stringReceived = false;
  }

  // Calibration
//   sensorCalibration();
  //Serial.println(analogRead(A0));
//  delay(100);
//      movementCalibration('d');
//      wall_alignment();
  //    while(1){}

  //Path Test
  //     delay(100);
  //     forward(40.0,false);
  //   delay(100);
  //   forward(10.0);
  //   delay(100);
  //   forward(10.0);
  //     delay(100);
  //     right(90.0,false);
  //     delay(100);
  //     forward(40.0,false);
  //     delay(100);
  //   forward(10.0);
  //   delay(100);
  //     left(90.0,false);
  //     delay(100);
  //     forward(40.0,false);
  //   delay(100);
  //   forward(10.0);
  //   delay(100);
  //   forward(10.0);
  //   delay(100);
  //   forward(10.0);
  //   delay(100);
  //   left(90.0);
  //   delay(100);
  //   forward(10.0);
  //   delay(100);
  //   forward(10.0);
  //   delay(100);
  //   right(90.0);
  //   delay(100);
  //   forward(10.0);
  //   delay(100);
  //   forward(10.0);
  //   delay(100);
  //  while(1){}
}


void wall_alignment()
{
  double fl, fr, angle;

  fl = final_MedianRead(irFL);
  fr = final_MedianRead(irFR);

  distAlign();
  distAlign();

  if (millis() - timeStartAlign <= 2000 && fabs(fl - fr) > 0.4 && fabs(fl - fr) < 6.0) {

    if (fl > fr) {
      angle = asin((fl - fr) / distBtwnFLR);
      //      Serial.println(angle);
      right(RAD_TO_DEG * angle, true);
    }
    else {
      angle = asin((fr - fl) / distBtwnFLR);
      //      Serial.println(angle);
      left(RAD_TO_DEG * angle, true);
    }
    wall_alignment();
  }

  distAlign();
}

void distAlign() {
  double dist;

  dist = final_MedianRead(irFL);

  if (dist > 15.5) {
    dist = final_MedianRead(irFR);
    if (dist > 15.5) {
      dist = final_MedianRead(irFM);
      if (dist > 12.0) {
        return;
      }
    }
  }

  if (dist < 10.4) {
    //    Serial.println(10-fl);
    backward(10.4 - dist);
  }
  else if (dist > 10.4) {
    //    Serial.println(fl-10);
    forward(dist - 10.4, true);
  }
}


/*
      Motion Control ********************************************************************************************************************************
*/

void forward(double distance, boolean cal) {
  double  distanceTraversed, angular_error, v, w, setPoint, halfDist, topSpeed;
  float distanceL, distanceR, threshold;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  setPoint = 0.0;
  halfDist = distance / 2;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();

  if (cal) {
    v = 130;
    threshold = 0.4;
  }
  else {
    v = 340;
    threshold = 1.6;
  }

  while ( fabs(distance - distanceTraversed) > threshold) {
    angular_error = distanceL - distanceR;
    //Serial.print("   Forward error: ");
    //Serial.println(distanceL - distanceR);
    //Serial.print(" ");
    //Serial.print("   Distance left: ");
    //Serial.println(distanceTraversed);

    PID_angular.Compute();

    if (!cal) {
      if (fabs(distance - distanceTraversed) < halfDist && v >= 300) {
        v = (topSpeed - 300) * cos((distanceTraversed - halfDist) * (22 / 7) / (halfDist * 2)) + 300;
      }
      else if (v < 375) {
        v += 0.02;
        topSpeed = v;
      }
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.print(v / 100);
    //    Serial.print(" ");
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);

    md.setSpeeds((-v - w) * 0.96, v - w);

    setPoint -= 0.000045;

    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    //    Serial.print("   DistanceL: ");
    //    Serial.print(distanceL);
    //    Serial.print(" ");
    //    Serial.print("   DistanceR: ");
    //    Serial.print(distanceR);
    //    Serial.print(" ");
    distanceTraversed = (distanceL + distanceR) / 2;
  }

  md.setM1Brake(400);
  delayMicroseconds(1870);
  md.setM2Brake(400);
}


void backward(double distance) {
  double  distanceTraversed, angular_error, v, w, setPoint, halfDist, topSpeed;
  float distanceL, distanceR;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  setPoint = 0.0;
  halfDist = distance / 2;
  v = 130;
  w = 0;


  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();

  while ( fabs(distance - distanceTraversed) > 0.3) {
    angular_error = distanceL - distanceR;

    //Serial.print("   Back error: ");
    //Serial.print(distanceL - distanceR);
    //Serial.print(" ");
    //Serial.print("   Distance left: ");
    //Serial.println(distanceTraversed);
    //
    //    if (fabs(distance - distanceTraversed) < halfDist && v >= 250) {
    //      v = (topSpeed - 250) * cos((distanceTraversed - halfDist) * (22 / 7) / (halfDist * 2)) + 250;
    //    }
    //    else if (v < 350) {
    //      v += 0.02;
    //      topSpeed = v;
    //    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.print(v / 100);
    //    Serial.print(" ");
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);

    md.setSpeeds((v - w) * 0.98, -v - w);

    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();
    //    Serial.print("   DistanceL: ");
    //    Serial.print(distanceL);
    //    Serial.print(" ");
    //    Serial.print("   DistanceR: ");
    //    Serial.print(distanceR);
    //    Serial.print(" ");
    distanceTraversed = -(distanceL + distanceR) / 2;
  }
  md.setM1Brake(400);
  delayMicroseconds(2000);
  md.setM2Brake(400);
}

void left(double angle, boolean cal) {
  double distanceL, distanceR, angleTraversed, angular_error, v, w, setPoint, topSpeed, halfAngle;
  float threshold;
  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  halfAngle = angle / 2;
  setPoint = 0.0;
  angular_error = 0;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, turn_kp, turn_ki, turn_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();

  if (cal) {
    v = 100;
    threshold = 0.4;
  }
  else {
    v = 120;
    threshold = 3.0;
  }

  while ( angle - fabs(angleTraversed) > threshold) {
    //    Serial.print("   L: ");
    //    Serial.print(distanceL);
    //    Serial.print("   R: ");
    //    Serial.print(distanceR);
    //    Serial.print("   angle left: ");
    //    Serial.print(fabs(angle - fabs(angleTraversed)) / 90);
    //    Serial.print(" ");
    //    Serial.print("   Left error: ");
    //    Serial.print(angular_error);
    PID_angular.Compute();

    if (!cal) {
      if (fabs(angle - fabs(angleTraversed)) < halfAngle && v > 80) {
        v = (topSpeed - 80) * cos((angleTraversed - halfAngle) * (22 / 7) / angle) + 80;
      }
      else if (v < 250) {
        v = v + 0.2;
        topSpeed = v;
      }
    }
    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.println(v / 125);
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);

    md.setSpeeds((v - w) * 0.95, v + w);

    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
    angleTraversed = (angleTraversed * 4068) / 71;

    angular_error = distanceL + distanceR; //shift forward |right| > |left|
  }
  md.setM1Brake(400);
  delay(2);
  md.setM2Brake(400);

}

void right(double angle, boolean cal) {
  double distanceL, distanceR, angleTraversed, angular_error, v, w, setPoint, topSpeed, halfAngle;
  float threshold;
  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  halfAngle = angle / 2;
  setPoint = 0.0;
  angular_error = 0;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, turn_kp, turn_ki, turn_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();

  if (cal) {
    v = 100;
    threshold = 0.4;
  }
  else {
    v = 120;
    threshold = 3.151;
  }

  while ( angle - fabs(angleTraversed) > threshold) { //if over, increase threshold

    //    Serial.print("   L: ");
    //    Serial.print(distanceL);
    //    Serial.print("   R: ");
    //    Serial.print(distanceR);
    //    Serial.print("   angle left: ");
    //    Serial.print(fabs(angle - fabs(angleTraversed)) / 90);
    //    Serial.print(" ");
    //    Serial.print("   right error: ");
    //    Serial.print(angular_error);

    PID_angular.Compute();

    if (!cal) {
      if (fabs(angle - fabs(angleTraversed)) < halfAngle && v > 80) {
        v = (topSpeed - 80) * cos((-angleTraversed - halfAngle) * (22 / 7) / angle) + 80;
      }
      else if (v < 250) {
        v = v + 0.2;
        topSpeed = v;
      }
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.println(v / 125);
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);
    md.setSpeeds(-(v - w) * 0.95, w - v);

    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
    angleTraversed = (angleTraversed * 4068) / 71;

    angular_error = distanceL + distanceR; //overshoot -> |left| > |right|
  }
  md.setM1Brake(400);
  md.setM2Brake(400);
}

void readEncoder1() {
  en.rencoder1();
}

void readEncoder2() {
  en.rencoder2();
}

/*
   Sensor Reading ***************************************************************************************
   short 7 <= x <= 27
   front adjustment 5.5, 1.5, 5.5
   left adjustment 5, 3
   right adjustment 11
*/

void sendInfo(char c) {
  String execute = String("");
  if (c == 'e') {
    execute = String("X") + String('x');
  }
  else {
    execute = String("X") + String(c);
  }
  String resultFL = String(final_MedianRead(irFL)) + String(",");
  String resultFM = String(final_MedianRead(irFM)) + String(",");
  String resultFR = String(final_MedianRead(irFR)) + String(",");
  String resultLF = String(final_MedianRead(irLF)) + String(",");
  String resultLM = String(final_MedianRead(irLM)) + String(",");
  String resultRM = String(final_MedianRead(irRM));
  Serial.println(execute + resultFL + resultFM + resultFR + resultLF + resultLM + resultRM);
}

double final_MedianRead(int tpin) {
  double x[9];

  //  if (tpin == irFM)
  //    return Ultra_Sensor() * 1.015 + 0.5;

  for (int i = 0; i < 9; i ++) {
    x[i] = distanceFinder(tpin);
  }
  insertionsort(x, 9);
  
  return x[5];
}

/*
   fs 7-15 1 block, 16-22 2 blocks
*/
double distanceFinder(int pin)
{
  double dis = 0.0;
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

void PWM_Mode_Setup()
{
  int URPWM = 6; // PWM Output 0-25000US,Every 50US represent 1cm 3
  int URTRIG = 12; // PWM trigger pin 5
  uint8_t EnPwmCmd[4] = {0x44, 0x02, 0xbb, 0x01};
  pinMode(URTRIG, OUTPUT);                    // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG, HIGH);                 // Set to HIGH

  pinMode(URPWM, INPUT);                      // Sending Enable PWM mode command

  for (int i = 0; i < 4; i++)
  {
    Serial.write(EnPwmCmd[i]);
  }
}

double Ultra_Sensor()
{ // a low pull on pin COMP/TRIG  triggering a sensor reading
  int URPWM = 6; // PWM Output 0-25000US,Every 50US represent 1cm 3 yellow
  int URTRIG = 12; // PWM trigger pin 5 green
  double Distance = 0;
  digitalWrite(URTRIG, LOW);
  digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses

  unsigned long DistanceMeasured = pulseIn(URPWM, LOW);

  if (DistanceMeasured >= 10200)
  { // the reading is invalid.
    //Serial.println("Invalid");
    Distance = -1;
  }
  else
  {
    Distance = DistanceMeasured / 50.0;       // every 50us low level stands for 1cm
    // Serial.print("Distance=");
    // Serial.print(Distance);
    // Serial.println("cm");
  }

  return Distance;
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
    //    Serial.print("Serial event char: ");
    //            Serial.println(inChar);
    if (inChar == '\n') {
      int i = 2;
      while (instructionString[i] != '\n') {
        stringValueReceived += instructionString[i];
        i++;
      }
      if (stringValueReceived.length() == 0) {
        stringValueReceived = String("10");
      }
      intValueReceived = stringValueReceived.toInt();
      stringReceived = true;
    }
  }
}

void sensorCalibration() {
  sendInfo('e');
  delay(100);
}

void movementCalibration(char c) {
  if (c == 'a') {
    left(90.0, false);
    delay(200);
  }
  else if (c == 'd') {
    right(90.0, false);
    delay(200);
  }
  else if (c == 'w') {
    forward(80, false);
    while (1) {}
  }
  else if (c == 's') {
    for ( int i = 0; i < 8; i++) {
      delay(200);
      forward(10.0, false);
    }
    while (1) {}
  }

}

