//#include <ultasonic.h>

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
#define angular_ki 60.0
#define angular_kd 1.0

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
void getCmd();

/**
   Variable Declarations
*/
String instructionString = "";
String feedbackString = "";
bool stringReceived = false;
double d;

DualVNH5019MotorShield md;
Encoder en(enA1, enB1, enA2, enB2);

SharpIR sensorFR(irFR, 1, 93, SHORT);  // (pin , no.reading b4 calculate mean dist, diff btw 2 consecutive measu taken as valid)
SharpIR sensorFL(irFL, 1, 93, SHORT);
//SharpIR sensorFM(irFM, 1, 93, SHORT);
SharpIR sensorLM(irLM, 1, 93, SHORT);
SharpIR sensorLF(irLF, 1, 93, SHORT);
SharpIR sensorRM(irRM, 1, 93, LONG);

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

  //  Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(enA1), readEncoder1, FALLING);
  PCintPort::attachInterrupt(enA2, readEncoder2, FALLING);

  PWM_Mode_Setup();

  //  emergency brake
  //  PCintPort::attachInterrupt(A5, Export_Sensors, RISING);

  // Serial data
  instructionString.reserve(200);
}

void loop()
{
  //  if (stringReceived)  {
  //    if (instructionString == "w")
  //      forward(10.0);
  //    else if (instructionString == "s")
  //      backward(10.0);
  //    else if (instructionString == "a")
  //      left(90.0);
  //    else if (instructionString == "d")
  //      right (90.0);
  //    else
  //      instructionString = "";
  //    Serial.print("Executed " + instructionString);
  //    instructionString = "";
  //    stringReceived = false;
  //  }
  // Export_Sensors();
 wall_alignment();
 // d=Ultra_Sensor();
  //Serial.print(d);
  //Serial.print("  ");
  delay(200);
  //forward(100.0);
  //curved(40.0);
 // forward(30.0);
  // md.setSpeeds(400, 400);
  //  backward(100.0);
  //while (1) {}
}


void wall_alignment()
{
  double fl, fm, fr, lm, l_m_distance = 6, m_r_distance = 4, l_r_distance = 10, y1, y2, x;
  boolean aligned = false, turn_right=false, turn_left=false;
  int count = 0;

  while (count < 3 && aligned == false)
  {
    
    fl = final_MedianRead(irFL);
    fm = final_MedianRead(irFM);
    fr = final_MedianRead(irFR);
    Serial.print(fl);
    Serial.print("  ");
    Serial.print(fm);
    Serial.print("  ");
    Serial.print(fr);
    Serial.print("  ");

    if (fl-fr<=fabs(2) || fl-fm<=fabs(1) || fm-fr<=fabs(1))
     aligned =true;
      
    if (aligned != true )
    {
      Serial.print("inside if ");
      if (fl > fr)
      {
        y2 = fl - fr;
        y1 = fm - fr;
        x = m_r_distance;
        turn_right = true;
      }
      else if (fr < fl)
      {
        y2 = fr - fl;
        y1 = fm - fl;
        x = l_m_distance;
        turn_left = true;
      }
      
      double angle_error = min(atan2(y2, l_r_distance), atan2(y1, x));
      angle_error = angle_error*360*7/22;
      Serial.print("angle error ");
      Serial.print(turn_right);
      Serial.print("  ");

      if (turn_right)
      {
        right(angle_error);
        turn_right = false;
      }
      else if (turn_left)
      {
        left(angle_error);
        turn_left = false;
      }
      count++;
    }

   
  }
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
    //Serial.print("   Forward error: ");
    //Serial.println(angular_error);
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

  while ( fabs(distance - distanceTraversed) > 0.5) {
    distanceL += 2 * (22 / 7) * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * (22 / 7) * wheelRadius * en.getMotor2Revs();

    distanceTraversed = -(distanceL + distanceR) / 2;

    angular_error = distanceL - distanceR;

    //    Serial.print("   Back error: ");
    //    Serial.print(angular_error);
    //    Serial.print(" ");
    //    Serial.print("   Distance left: ");
    //    Serial.print(fabs(distance - distanceTraversed) / 100);

    PID_angular.Compute();
    if (fabs(distance - distanceTraversed) < distance / 2 && v > 180) {
      v = v - 0.001 * v;
    }
    else if (v < 350) {
      v = v + 0.001 * v;
    }
    w = w - 0.001 * w;

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
    //    Serial.print("   Left error: ");
    //    Serial.println(angular_error);
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
    //    Serial.print("   right error: ");
    //    Serial.println(angular_error);

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



  /*  while ( fabs(90 - fabs(angleTraversed)) > 0.8 ) {
     distanceL += 2 * Pi * wheelRadius * en.getMotor1Revs();
     distanceR += 2 * Pi * wheelRadius * en.getMotor2Revs();

     angleTraversed = (distanceR - distanceL) / distanceBetweenWheels;
     angleTraversed = (angleTraversed * 4068) / 71;

     angular_error = distanceL + distanceR;

     //    Serial.print("   L: ");
     //    Serial.print(distanceL);
     //    Serial.print("   R: ");
     //    Serial.print(distanceR);
     Serial.print("   angle left: ");
     Serial.print(fabs(90 - fabs(angleTraversed)) / 90);
     Serial.print(" ");
     Serial.print("   Left error: ");
     Serial.print(angular_error);
     PID_angular.Compute();
     if (fabs(90 - fabs(angleTraversed)) < 80.0 && v > 100) {
       v = v - 0.4;
     }
     else if (v < 250) {
       v = v + 0.4;
     }

     Serial.print(" ");
     Serial.print("   v: ");
     Serial.println(v / 125);
     //    Serial.print("   w: ");
     //    Serial.println(w);

     md.setSpeeds(cons_l , v + 2*w);
    }*/

  //  md.setBrakes(400, 400);



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
      break;
    case irFL:
      dis = sensorFL.distance();
      break;
    case irFM:
      //dis = sensorFM.distance();
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

void avoidObs() {
  double fl, fm, fr, lm;
  fl = final_MedianRead(irFL);
  fm = final_MedianRead(irFM);
  fr = final_MedianRead(irFR);

  while (fl > 16.5 && fm > 16.5 && fr > 16.5) {
    forward(10.0);
    fl = final_MedianRead(irFL);
    fm = final_MedianRead(irFM);
    fr = final_MedianRead(irFR);
  }

  right(90.0);

  lm = final_MedianRead(irLM);

  while (lm <= 16.5) {
    forward(10.0);
    lm = final_MedianRead(irLM);
  }

  forward(10.0);
  left(90.0);

  lm = final_MedianRead(irLM);

  while (lm <= 16.5) {
    forward(10.0);
  }

  forward(10.0);
  left(90.0);
  forward(10.0);


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
  int URPWM = 6; // PWM Output 0-25000US,Every 50US represent 1cm 3
  int URTRIG = 12; // PWM trigger pin 5
  double Distance = 0;
  digitalWrite(URTRIG, LOW);
  digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses

  unsigned long DistanceMeasured = pulseIn(URPWM, LOW);

  if (DistanceMeasured >= 10200)
  { // the reading is invalid.
    //Serial.println("Invalid");
    Distance=-1;
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

