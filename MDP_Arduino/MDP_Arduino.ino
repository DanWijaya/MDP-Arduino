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

#define irFM A0
#define irFL A4
#define irFR A5
#define irLF A1
#define irLM A3
#define irRM A2
#define distBtwnFLR 17

/**
   Function Declarations
*/

void forward(double);
void backward(double);
void left(double);
void right(double);
void readEncoder1();
void readEncoder2();
void getCmd();

float const pi = 3.14159;
/**
   Variable Declarations
*/
String instructionString = "";
String feedbackString = "";
String stringValueReceived = "";
int intValueReceived = 0;
bool stringReceived = false;
double d;
long timeStartAlign = 0;

DualVNH5019MotorShield md;
Encoder en(enA1, enB1, enA2, enB2);

SharpIR sensorFR(irFR, 10, 10, FR);  // (pin , no.reading b4 calculate mean dist, diff btw 2 consecutive measu taken as valid)
SharpIR sensorFL(irFL, 10, 10, FL);
SharpIR sensorLM(irLM, 10, 10, LM);
SharpIR sensorLF(irLF, 10, 10, LF);
SharpIR sensorRM(irRM, 10, 10, RM);


void setup()
{
  Serial.begin(115200);
  md.init();
  en.init();

  pinMode(irFR, INPUT);
  pinMode(irFL, INPUT);
  //pinMode(irFM, INPUT);
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

  //  Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(enA2), readEncoder2, FALLING);
  PCintPort::attachInterrupt(enA1, readEncoder1, FALLING);

  PWM_Mode_Setup();

  // Serial data
  instructionString.reserve(200);
}

void loop()
{
  //  if (stringReceived)  {
  //    if (instructionString[0] == 'm') {
  //      if (instructionString[1] == 'w') {
  //        forward(intValueReceived,false);
  //        sendInfo(instructionString[1]);
  //      }
  //      else if (instructionString[1] == 's') {
  //        backward(intValueReceived);
  //        sendInfo(instructionString[1]);
  //      }
  //      else if (instructionString[1] == 'a') {
  //        left(90.0, false);
  //        sendInfo(instructionString[1]);
  //      }
  //      else if (instructionString[1] == 'd') {
  //        right (90.0, false);
  //        sendInfo(instructionString[1]);
  //      }
  //      else if (instructionString[1] == 'l') {
  //        timeStartAlign = millis();
  //        wall_alignment();
  //        sendInfo(instructionString[1]);
  //      }
  //      else
  //        instructionString = "";
  //    }
  //    else if (instructionString.equals(String("start\n"))) {
  //      sendInfo('e');
  //    }
  //    instructionString = "";
  //    stringValueReceived = "";
  //    stringReceived = false;
  //  }

  //  Sensor Calibration

  //Serial.println(sensorRM.cm());
  sendInfo('e');
  delay(100);

  //Movement Calibration

  //      left(90.0,false);
  //        right(90.0,false);
  //      forward(100);
  //    for ( int i = 0; i < 10; i++) {
  //      delay(100);
  //      forward(10.0);
  //    }
  //
  //    while (1) {}
  //     Wall Alignment Test
  //  wall_alignment();

  //  right(90.0);
  //  while(1){}
  //d=Ultra_Sensor();
  //Serial.print(d);
  //Serial.print("  ");
  //  delay(200);

  //  sendInfo('x');

  //Path Test
  //   delay(100);
  //   forward(10.0);
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
  //   left(90.0);
  //   delay(100);
  //   forward(10.0);
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


}


void wall_alignment()
{
  double fl, fr, angle;

  fl = final_MedianRead(irFL);
  fr = final_MedianRead(irFR);
  //  Serial.print("fl: ");
  //  Serial.print(fl);
  //  Serial.print("   fm: ");
  //  Serial.print(fm);
  //  Serial.print("   fr: ");
  //  Serial.println(fr);

  if (millis() - timeStartAlign <= 2000 && fabs(fl - fr) > 0.4) {

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
  double fm;

  fm = final_MedianRead(irFM);

  if (fm < 5.5) {
    //    Serial.println(10-fl);
    backward(5.5 - fm);
  }
  else if (fm > 5.5) {
    //    Serial.println(fl-10);
    forward(fm - 5.5, true);
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
    v = 120;
    threshold = 1.4;
  }
  else {
    v = 280;
    threshold = 1.4;
  }

  while ( fabs(distance - distanceTraversed) > threshold) {
    angular_error = distanceL - distanceR;
    //Serial.print("   Forward error: ");
    //Serial.println(distanceL - distanceR);
    //Serial.print(" ");
    //Serial.print("   Distance left: ");
    //Serial.println(distanceTraversed);

    PID_angular.Compute();

    if (fabs(distance - distanceTraversed) < halfDist && v >= 250) {
      v = (topSpeed - 250) * cos((distanceTraversed - halfDist) * (22 / 7) / (halfDist * 2)) + 250;
    }
    else if (v < 375) {
      v += 0.02;
      topSpeed = v;
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.print(v / 100);
    //    Serial.print(" ");
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);

    md.setSpeeds((-v - w), v - w);
    setPoint -= 0.00004;

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
  delayMicroseconds(2500);
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
  v = 280;
  w = 0;


  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();

  while ( fabs(distance - distanceTraversed) > 0.5) {
    angular_error = distanceL - distanceR;

    //Serial.print("   Back error: ");
    //Serial.print(distanceL - distanceR);
    //Serial.print(" ");
    //Serial.print("   Distance left: ");
    //Serial.println(distanceTraversed);

    if (fabs(distance - distanceTraversed) < halfDist && v >= 250) {
      v = (topSpeed - 250) * cos((distanceTraversed - halfDist) * (22 / 7) / (halfDist * 2)) + 250;
    }
    else if (v < 350) {
      v += 0.02;
      topSpeed = v;
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.print(v / 100);
    //    Serial.print(" ");
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);

    md.setSpeeds((v - w) * 0.85, -v - w);

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
    v = 80;
    threshold = 0.5;
  }
  else {
    v = 150;
    threshold = 2.7;
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
    if (fabs(angle - fabs(angleTraversed)) < halfAngle && v > 80) {
      v = (topSpeed - 80) * cos((angleTraversed - halfAngle) * (22 / 7) / angle) + 80;
    }
    else if (v < 250) {
      v = v + 0.2;
      topSpeed = v;
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.println(v / 125);
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);

    md.setSpeeds((v - 20 - w) * 0.85, v - 20 + w);

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
  v = 150;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, turn_kp, turn_ki, turn_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();

  if (cal) {
    v = 80;
    threshold = 0.5;
  }
  else {
    v = 150;
    threshold = 3.2;
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

    if (fabs(angle - fabs(angleTraversed)) < halfAngle && v > 80) {
      v = (topSpeed - 80) * cos((-angleTraversed - halfAngle) * (22 / 7) / angle) + 80;
    }
    else if (v < 250) {
      v = v + 0.2;
      topSpeed = v;
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.println(v / 125);
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);
    md.setSpeeds((-(v - 20) - w) * 0.85, w - v - 20);

    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
    angleTraversed = (angleTraversed * 4068) / 71;

    angular_error = distanceL + distanceR; //overshoot -> |left| > |right|
  }
  md.setM1Brake(400);
  delay(2);
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
  double x[15];


  if (tpin == irFM)
    return Ultra_Sensor() * 1.015 + 0.5;

  for (int i = 0; i < 15; i ++) {
    x[i] = distanceFinder(tpin);
  }
  insertionsort(x, 15);
  return x[7];
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
      dis = Ultra_Sensor();
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


