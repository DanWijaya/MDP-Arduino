#include "Encoder.h"
#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"
#include "PID_v1.h"
//#include "FastRunningMedian.h"
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
//long d = 0;

DualVNH5019MotorShield md;
Encoder en(enA1, enB1, enA2, enB2);

SharpIR sensorFR(irFR, 15, 50, FR);  // (pin , no.reading b4 calculate mean dist, diff btw 2 consecutive measu taken as valid)
SharpIR sensorFL(irFL, 15, 50, FL);
SharpIR sensorLM(irLM, 15, 50, LM);
SharpIR sensorLF(irLF, 15, 50, LF);
SharpIR sensorRM(irRM, 15, 50, RM);
SharpIR sensorFM(irFM, 15, 50, FM);


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

  //  PWM_Mode_Setup();

  // Serial data
  instructionString.reserve(200);
}


void loop()
{
  if (stringReceived)  {
    if (instructionString[0] == 'm') {
      switch (instructionString[1]) {
        case 'w':
          forward(intValueReceived, false);
          distAlign();
          sendInfo(instructionString[1]);
          break;

        case 's':
          backward(intValueReceived);
          sendInfo(instructionString[1]);
          break;

        case 'a':
          left(90.0, false);
          distAlign();
          sendInfo(instructionString[1]);
          break;

        case 'd':
          right (90.0, false);
          distAlign();
          sendInfo(instructionString[1]);
          break;

        case 'l':
          timeStartAlign = millis();
          wall_alignment();
          sendInfo(instructionString[1]);
          break;

        case 't':
          forward(intValueReceived, false);
          distAlign();
          sendInfo('g');
          break;

        case 'f':
          left(90.0, false);
          distAlign();
          sendInfo('g');
          break;

        case 'h':
          right (90.0, false);
          distAlign();
          sendInfo('g');
          break;

        default:  instructionString = "";
      }
    }
    else if (instructionString.equals(String("start\n"))) {
      sendInfo('e');
    }
    instructionString = "";
    stringValueReceived = "";
    stringReceived = false;
  }

  // Calibration
  //     sensorCalibration();
  //    delay(100);
  //      movementCalibration('a');
  // wall_alignment();
  //  while(1){}

  //Path Test
  //  delay(100);
  //  forward(20.0, false);
  //  delay(100);
  //  left(90.0, false);
  //  delay(100);
  //  forward(10.0, false);
  //  delay(100);
  //  right(90.0, false);
  //  delay(100);
  //  forward(10.0, false);
  //  delay(100);
  //  forward(10.0, false);
  //  while (1) {}
}


void wall_alignment()
{
  double fl, fr, fm, angle;

  fl = final_MedianRead(irFL);
  fr = final_MedianRead(irFR);

  distAlign();
  distAlign();

  if (millis() - timeStartAlign <= 2500 && fabs(fl - fr) > 0.4 && fabs(fl - fr) < 8.5) {

    if (fl > fr) {
      angle = atan2((fl - fr), distBtwnFLR);
      right(RAD_TO_DEG * angle, true);
    }
    else {
      angle = atan2((fr - fl), distBtwnFLR);
      left(RAD_TO_DEG * angle, true);
    }
    wall_alignment();
  }

  distAlign();
}

void distAlign() {
  double fl, fm, fr, dist;
  fl = 0.0;
  fr = 0.0;
  fm = 0.0;
  dist = 0.0;

  fm = final_MedianRead(irFM);
  fl = final_MedianRead(irFL);
  fr = final_MedianRead(irFR);

  if (fl <= 15.5 && fl >= 6.0) {
    dist = fl;
    dist -= 5.5;
  }
  else if (fm <= 12.0 && fm >= 6.0) {
    dist = fm;
    dist -= 2.5;
  }
  else if (fr <= 15.5 && fr >= 6.0) {
    dist = fr;
    dist -= 5.5;
  }
  else {
    return;
  }

  if (dist < 5.25) {
    backward(5.25 - dist);
  }
  else if (dist > 5.25) {
    forward(dist - 5.25, true);
  }
}


/*
      Motion Control ********************************************************************************************************************************
*/

void forward(double distance, boolean cal) {
  double  distanceTraversed, angular_error, v, w, setPoint, halfDist, topSpeed;
  float distanceL, distanceR, threshold;
  distanceTraversed = 0;
  distanceL = 0.0;
  distanceR = 0.0;
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
    v = 200;
    threshold = 0.4;
  }
  else {
    v = 340;
    threshold = 1.7;
  }

  while ( fabs(distance - distanceTraversed) > threshold) {
    angular_error = distanceL - distanceR;

    PID_angular.Compute();

    if (!cal) {
      if (fabs(distance - distanceTraversed) < halfDist && v >= 340) {
        v = (topSpeed - 340) * cos((distanceTraversed - halfDist) * (22 / 7) / (halfDist * 2)) + 340;
      }
      else if (v < 386) {
        v += 0.05;
        topSpeed = v;
      }
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.print(v / 100);
    //    Serial.print(" ");
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);

    md.setSpeeds((-v - w) * 0.97, v - w);

    setPoint -= 0.000055;

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
  delayMicroseconds(1700);
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
  v = 200;
  w = 0;


  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();

  while ( fabs(distance - distanceTraversed) > 0.3) {
    angular_error = distanceL - distanceR;

    md.setSpeeds((v - w) * 0.98, -v - w);

    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

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
    v = 130;
    threshold = 0.45;
  }
  else {
    v = 250;
    threshold = 5.8;
  }

  while ( angle - fabs(angleTraversed) > threshold) {
    PID_angular.Compute();

    if (!cal) {
      if (fabs(angle - fabs(angleTraversed)) < halfAngle && v > 250) {
        v = (topSpeed - 250) * cos((angleTraversed - halfAngle) * (22 / 7) / angle) + 250;
      }
      else if (v < 280) {
        v += 0.05;
        topSpeed = v;
      }
    }
    md.setSpeeds((v - w) * 0.95, v + w);

    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
    angleTraversed = angleTraversed * 57.2957795;

    angular_error = distanceL + distanceR; //shift forward |right| > |left|
  }
  md.setBrakes(400, 400);
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
    v = 130;
    threshold = 0.45;
  }
  else {
    v = 250;
    threshold = 5.3;
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
      if (fabs(angle - fabs(angleTraversed)) < halfAngle && v > 250) {
        v = (topSpeed - 250) * cos((-angleTraversed - halfAngle) * (22 / 7) / angle) + 250;
      }
      else if (v < 280) {
        v += 0.05;
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
    angleTraversed = angleTraversed * 57.2957795;

    angular_error = distanceL + distanceR; //overshoot -> |left| > |right|
  }
  md.setBrakes(400, 400);
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
  String execute = String("X");

  if (c == 'g') {
    execute += String(c);
    Serial.println(execute);
  }
  else {
    if (c == 'e') {
      execute += String('x');
    }
    else {
      execute += String(c);
    }
    String resultFL = String(final_MedianRead(irFL)) + String(",");
    String resultFM = String(final_MedianRead(irFM)) + String(",");
    String resultFR = String(final_MedianRead(irFR)) + String(",");
    String resultLF = String(final_MedianRead(irLF)) + String(",");
    String resultLM = String(final_MedianRead(irLM)) + String(",");
    String resultRM = String(final_MedianRead(irRM));
    Serial.println(execute + resultFL + resultFM + resultFR + resultLF + resultLM + resultRM);
  }
}

double final_MedianRead(int tpin) {
  double x[9];

  for (int i = 0; i < 9; i ++) {
    x[i] = distanceFinder(tpin);
  }

  insertionsort(x, 9);

  return x[4];
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
    forward(60, false);
    while (1) {}
  }
  else if (c == 's') {
    for ( int i = 0; i < 6; i++) {
      delay(200);
      forward(10.0, false);
    }
    while (1) {}
  }

}

