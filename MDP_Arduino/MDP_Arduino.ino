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
#define wheelRadius 3.07
#define distanceBetweenWheels 17

/*
   PID Values
*/
#define angular_kp 65.0
#define angular_ki 60.0
#define angular_kd 1.0

#define turn_kp 52.0
#define turn_ki 50.0
#define turn_kd 1.0


/*
   Sensor Settings
*/

#define LONG 20150
#define SHORT 1080

#define irFM A0
#define irFL A4
#define irFR A5
#define irLF A1
#define irLM A3
#define irRM A2
#define distBtwnLR 16.2
#define distBtwnLM 8.4
#define distBtwnMR 7.8
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

DualVNH5019MotorShield md;
Encoder en(enA1, enB1, enA2, enB2);

SharpIR sensorFR(irFR, 10, 10, SHORT);  // (pin , no.reading b4 calculate mean dist, diff btw 2 consecutive measu taken as valid)
SharpIR sensorFL(irFL, 10, 10, SHORT);
SharpIR sensorLM(irLM, 10, 10, SHORT);
SharpIR sensorLF(irLF, 10, 10, SHORT);
SharpIR sensorRM(irRM, 10, 10, LONG);


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
  // digitalWrite(irFM, LOW);
  digitalWrite(irRM, LOW);
  digitalWrite(irLM, LOW);
  digitalWrite(irLF, LOW);

  Serial.flush();

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
//          wall_alignment();
          forward(intValueReceived);
          sendInfo(instructionString[1]);
        }
        else if (instructionString[1] == 's') {
//          wall_alignment();
          backward(intValueReceived);
          sendInfo(instructionString[1]);
        }
        else if (instructionString[1] == 'a') {
//          wall_alignment();
          left(90.0);
          sendInfo(instructionString[1]);
        }
        else if (instructionString[1] == 'd') {
//          wall_alignment();
          right (90.0);
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
//  delay(200);
  //      Serial.println(analogRead(A4));
  //    Serial.println(sensorFR.distance());
  //     avoidObs();
  //    Export_Sensors();
  //      left(90.0);

  //    else if (instructionString == "d")
  //      right (90.0);
  //    else
  //      instructionString = "";
  //    Serial.print("Executed " + instructionString);
  //    instructionString = "";
  //    stringReceived = false;
  //  }


  /*
     Wall Alignment Test
  */
//  wall_alignment();
  //d=Ultra_Sensor();
  //Serial.print(d);
  //Serial.print("  ");
  //delay(200);

  //sendInfo('x');

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

  //    left(90.0);
  //    delay(100);
  //    right(90.0);
  //    delay(100);
//      while (1) {}


}


void wall_alignment()
{
  double fl, fm, fr, angle, sensorA, sensorB;
  uint8_t choice = 0;
  fl = final_MedianRead(irFL);
  fm = final_MedianRead(irFM);
  fr = final_MedianRead(irFR);
  Serial.print("fl: ");
  Serial.print(fl);
  Serial.print("   fm: ");
  Serial.print(fm);
  Serial.print("   fr: ");
  Serial.println(fr);
  if (fl != -1) {
    if (fr != -1) {
      sensorA = fl;
      sensorB = fr;
      choice = 0;
    }
    else {
      sensorA = fl;
      sensorB = fm;
      choice = 1;
    }
  }
  else if (fr != -1) {
    sensorA = fm;
    sensorB = fr;
    choice = 2;
  }
  else {
    Serial.println("return**********************************");
    return;
  }

  if (!(sensorA <= 21.0 && sensorA >= 9.0) && !(sensorB <= 21.0 && sensorB >= 9.0)) {
    Serial.println("return**********************************");
    return;
  }
  else {
    if (sensorA == sensorB) {
      if (sensorA == 17.0) {
        Serial.println("return**********************************");
        return;
      }
      else if (sensorA < 17.0) {
        backward(17.0 - sensorA);
        wall_alignment();
      }
      else if (sensorA > 17.0) {
        forward(sensorA - 17.0);
        wall_alignment();
      }
    }
    else {
      if (sensorA > sensorB) {
        if (choice == 0) {
          angle = asin((sensorA - sensorB) / distBtwnLR);
          right(RAD_TO_DEG * angle);
        }
        else if (choice == 1) {
          angle = asin((sensorA - sensorB) / distBtwnLM);
          right(RAD_TO_DEG * angle);
        }
        else {
          angle = asin((sensorA - sensorB) / distBtwnMR);
          right(RAD_TO_DEG * angle);
        }
      }
      else {
        if (choice == 0) {
          angle = asin((sensorB - sensorA) / distBtwnLR);
          left(RAD_TO_DEG * angle);
        }
        else if (choice == 1) {
          angle = asin((sensorB - sensorA) / distBtwnLM);
          left(RAD_TO_DEG * angle);
        }
        else {
          angle = asin((sensorB - sensorA) / distBtwnMR);
          left(RAD_TO_DEG * angle);
        }
      }
      wall_alignment();
    }
  }


  //  double fl, fm, fr, lm, l_m_distance = 5.5, m_r_distance = 4.8, l_r_distance = 10.3, y1, y2, x;
  //  boolean aligned = false, turn_right = false, turn_left = false;
  //
  //  int count = -9999;
  //
  //  while (count < 3 && aligned == false)
  //  {
  //
  //    fl = final_MedianRead(irFL);
  //    fm = final_MedianRead(irFM);
  //    fr = final_MedianRead(irFR);
  //
  //    //    Serial.print(fl);
  //    //    Serial.print("  ");
  //    //    Serial.print(fm);
  //    //    Serial.print("  ");
  //    //    Serial.print(fr);
  //    //    Serial.println("  ");
  //
  //    //    if (fl - fr <= fabs(2) || fl - fm <= fabs(1) || fm - fr <= fabs(1))
  //    //      aligned = true;
  //    //
  //    //    if (aligned != true )
  //
  //
  //    if (fl == fr || fl == fm || fm == fr)
  //      aligned = true;
  //
  //    if (aligned != false )
  //
  //    {
  //      // Serial.print("inside if ");
  //      if (fl > fr)
  //      {
  //        y2 = fl - fr;
  //        y1 = fm - fr;
  //        x = m_r_distance;
  //        turn_right = true;
  //      }
  //      else if (fr > fl)
  //      {
  //        y2 = fr - fl;
  //        y1 = fm - fl;
  //        x = l_m_distance;
  //        turn_left = true;
  //        // Serial.print("inside left ");
  //      }
  //
  //
  //      double angle_error = max(atan2(y2, l_r_distance), atan2(y1, x));
  //      angle_error = angle_error * 360 * 7 / 44;
  //      Serial.print("angle error ");
  //      Serial.println(angle_error);
  //      Serial.println(aligned);
  //      //      Serial.print("  ");
  //      //      Serial.print(y2);
  //      //       Serial.print("  ");
  //      //      Serial.println(x);
  //
  //
  //      if (turn_right)
  //      {
  //        right(angle_error);
  //        turn_right = false;
  //      }
  //      else if (turn_left)
  //      {
  //        left(angle_error);
  //        turn_left = false;
  //      }
  //      count++;
  //    }
  //  }
}


/*
      Motion Control ********************************************************************************************************************************
*/

void forward(double distance) {
  double  distanceTraversed, angular_error, v, w, setPoint;
  float distanceL, distanceR;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  setPoint = -0.05;
  v = 180;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();

  while ( distance - distanceTraversed > 0.8) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    //    Serial.print("   DistanceL: ");
    //    Serial.print(distanceL);
    //    Serial.print(" ");
    //    Serial.print("   DistanceR: ");
    //    Serial.print(distanceR);
    //    Serial.print(" ");
    distanceTraversed = (distanceL + distanceR) / 2;

    angular_error = distanceL - distanceR;
    Serial.print("   Forward error: ");
    Serial.println(distanceL - distanceR);
    //Serial.print(" ");
    //Serial.print("   Distance left: ");
    //Serial.println(distanceTraversed);

    PID_angular.Compute();

    if (fabs(distance - distanceTraversed) < distance / 2 && v > 200) {
      v -= 0.05;
    }
    else if (v < 350) {
      v = 1.001 * v;
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.print(v / 100);
    //    Serial.print(" ");
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);

    md.setSpeeds((-v - w) * 87 / 96, v - w);

  }

  md.setBrakes(400, 400);
}

void backward(double distance) {
  double distanceL, distanceR, distanceTraversed, angular_error, v, w, setPoint;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  setPoint = 0.05;
  v = 180;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();

  while ( fabs(distance - distanceTraversed) > 0.5) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();
    //    Serial.print("   DistanceL: ");
    //    Serial.print(distanceL);
    //    Serial.print(" ");
    //    Serial.print("   DistanceR: ");
    //    Serial.print(distanceR);
    //    Serial.print(" ");
    distanceTraversed = -(distanceL + distanceR) / 2;

    angular_error = distanceL - distanceR;

    //Serial.print("   Back error: ");
    //Serial.print(distanceL - distanceR);
    //Serial.print(" ");
    //Serial.print("   Distance left: ");
    //Serial.println(distanceTraversed);

    if (fabs(distance - distanceTraversed) < distance / 2 && v > 180) {
      v -= 0.05;
    }
    else if (v < 350) {
      v = 1.001 * v;
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.print(v / 100);
    //    Serial.print(" ");
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);

    md.setSpeeds(v - w, -v - w);

  }
  md.setBrakes(400, 400);
}

void left(double angle) {
  double distanceL, distanceR, angleTraversed, angular_error, v, w, setPoint;
  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  setPoint = 0.0;
  angular_error = 0;
  v = 130;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, turn_kp, turn_ki, turn_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();

  while ( angle - fabs(angleTraversed) > 2.8 ) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
    angleTraversed = (angleTraversed * 4068) / 71;

    angular_error = distanceL + distanceR; //shift forward |right| > |left|
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
    if (fabs(angle - fabs(angleTraversed)) < 40.0 && v > 110) {
      v = v - 0.5;
    }
    else if (v < 250) {
      v = v + 0.3;
    }

    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.println(v / 125);
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);

    md.setSpeeds((v - w) * 96 / 96, v + w);
  }
  md.setBrakes(400, 400);
}

void right(double angle) {
  double distanceL, distanceR, angleTraversed, angular_error, v, w, setPoint;
  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  setPoint = 0.0;
  angular_error = 0;
  v = 130;
  w = 0;

  PID PID_angular(&angular_error, &w, &setPoint, turn_kp, turn_ki, turn_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  //Flush the motor revs value;
  en.getMotor1Revs();
  en.getMotor2Revs();
  while ( angle - fabs(angleTraversed) > 2.8 ) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
    angleTraversed = (angleTraversed * 4068) / 71;

    angular_error = distanceL + distanceR; //overshoot -> |left| > |right|
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
    if (fabs(angle - fabs(angleTraversed)) < 40.0 && v > 110) {
      v = v - 0.5;
    }
    else if (v < 250) {
      v = v + 0.3;
    }
    //    Serial.print(" ");
    //    Serial.print("   v: ");
    //    Serial.println(v / 125);
    //    Serial.print("   w: ");
    //    Serial.println(w / 10);
    md.setSpeeds((-v - w) * 96 / 96, w - v);
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
  double x[21];


  if (tpin == irFM)
    return Ultra_Sensor();

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
//      if (dis < 7 || dis > 30) {
//        dis = -1;
//      }
      break;
    case irFL:
      dis = sensorFL.distance();
//      if (dis < 7 || dis > 30) {
//        dis = -1;
//      }
      break;
    case irFM:
      //dis = sensorFM.distance();
      dis = Ultra_Sensor();
      break;
    case irLF:
      dis = sensorLF.distance();
//      if (dis < 7 || dis > 30) {
//        dis = -1;
//      }
      break;
    case irLM:
      dis = sensorLM.distance();
//      if (dis < 7 || dis > 30) {
//        dis = -1;
//      }
      break;
    case irRM:
      dis = sensorRM.distance();
//      if (dis < 17 || dis > 65) {
//        dis = -1;
//      }
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
    Distance = DistanceMeasured / 50;       // every 50us low level stands for 1cm
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


/*
   Checklist *********************************************************************************
*/

void avoidObs() {
  double fl, fm, fr, lm;
  fl = final_MedianRead(irFL);
  fm = final_MedianRead(irFM);
  fr = final_MedianRead(irFR);
  //  Serial.print("1fl:");
  //  Serial.print(fl);
  //  Serial.print("   1fm:");
  //  Serial.print(fm);
  //  Serial.print("   1fr:");
  //  Serial.println(fr);
  while ((fl > 13 || fl < 8) && (fm > 13 || fm < 8) && (fr > 13 || fr < 8)) {
    forward(10.0);
    fl = final_MedianRead(irFL);
    fm = final_MedianRead(irFM);
    fr = final_MedianRead(irFR);
    //    Serial.print("fl:");
    //    Serial.print(fl);
    //    Serial.print("   fm:");
    //    Serial.print(fm);
    //    Serial.print("   fr:");
    //    Serial.println(fr);

    delay(50);
  }

  right(90.0);
  delay(50);

  forward(20.0);
  delay(50);
  left(90.0);


  lm = final_MedianRead(irLM);

  while (lm <= 16.5) {
    forward(10.0);
  }

  forward(10.0);
  left(90.0);
  forward(10.0);
}

void curved(double forward)
{
  double distanceL, distanceR, distanceTraversed, angular_error, v, w, setPoint;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  setPoint = 0;
  v = 110;
  w = 0;
  double Pi = 22 / 7;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  while ( forward - distanceTraversed > 0.5) {
    distanceL += 2 * Pi * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * Pi * wheelRadius * en.getMotor2Revs();

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
    if (fabs(forward - distanceTraversed) < forward / 2 && v > 180) {
      v = v - 0.0005 * v;
    }
    else if (v < 350) {
      v = v + 0.0005 * v;
    }
    w = w - 0.0005 * w;

    md.setSpeeds((-v - w) * 11 / 12, v - w);

  }

  v = fabs((-v - w) * 11 / 12) ;
  double cons_l = v;
  double  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  setPoint = 0;
  angular_error = 0;
  w = 0;
  md.setSpeeds(-cons_l / 1.5 , cons_l * 2.2 / 1.5);

  while (distanceL < 0.166 * Pi * 3 * distanceBetweenWheels / 2)
  {
    distanceL += 2 * Pi * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * Pi * wheelRadius * en.getMotor2Revs();
    Serial.print("   distanceL ");
    Serial.print(distanceL);
    Serial.print(" ");

  }
  md.setBrakes(400, 400);
}



