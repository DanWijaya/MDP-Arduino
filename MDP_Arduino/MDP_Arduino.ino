#include "Encoder.h"
#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

#define enA1    3
#define enB1    5
#define enA2    11
#define enB2    13

DualVNH5019MotorShield md;
Encoder en(enA1, enB1, enA2, enB2);
int rpm1 = 0;
int rpm2 = 0;
int currSpeed1 = 0;
int currSpeed2 = 0;
unsigned long lastMilliPrint = 0;

//void stopIfFault()
//{
//  if (md.getM1Fault())
//  {
//    Serial.println("M1 fault");
//    while(1);
//  }
//  if (md.getM2Fault())
//  {
//    Serial.println("M2 fault");
//    while(1);
//  }
//}


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

void readEncoder1() {
  en.rencoder1();
}

void readEncoder2() {
  en.rencoder2();
}

void loop()
{
  getCmd();
  getRpm();
}

void getCmd() {
  char cmd;
  if (!Serial.available())    return;
  delay(10);
  cmd = Serial.read();                                // get command byte
  Serial.flush();                                     // clean serial buffer


  //w   :   forward
  //s   :   reverse
  //a  :   left turn on the spot
  //d  :   right turn on the spot
  switch (cmd) {
    case 'w':
      if (currSpeed1 >= currSpeed2) {
        currSpeed2 = currSpeed1;
      }
      else {
        currSpeed1 = currSpeed2;
      }

      for ( ; currSpeed1 <= 400; currSpeed1 += 1) {
        md.setM1Speed(-currSpeed1);
        md.setM2Speed(currSpeed1);
        delay(2);
      }
      break;

    case 's':
      if (currSpeed1 <= currSpeed2) {
        currSpeed2 = currSpeed1;
      }
      else {
        currSpeed1 = currSpeed2;
      }

      for ( ; currSpeed1 >= -400; currSpeed1 -= 1) {
        md.setM1Speed(-currSpeed1);
        md.setM2Speed(currSpeed1);
        delay(2);
      }
      break;

    case 'a':
      md.setM1Speed(0);
      md.setM2Speed(0);
      for (currSpeed1 = 0, currSpeed2 = 0 ; currSpeed1 <= 400; currSpeed1 += 1, currSpeed2 -= 1) {
        md.setM1Speed(currSpeed1);
        md.setM2Speed(currSpeed2);
        delay(2);
      }
      delay(2);
      break;

    case 'd':
      md.setM1Speed(0);
      md.setM2Speed(0);
      for (currSpeed1 = 0, currSpeed2 = 0 ; currSpeed2 <= 400; currSpeed2 += 1, currSpeed1 -= 1) {
        md.setM1Speed(currSpeed1);
        md.setM2Speed(currSpeed2);
        delay(2);
      }
      delay(2);
      break;

    default:
      Serial.print("Speed set error: ");
      Serial.println(currSpeed1);
  }
}

void getRpm() {
  if ((millis() - lastMilliPrint) >= 150) {
    lastMilliPrint = millis();

    rpm1 = en.getMotor1RPM();
    rpm2 = en.getMotor2RPM();
    Serial.print("   RPM1: ");
    Serial.print(rpm1);
    Serial.print("   RPM2: ");
    Serial.print("\t");
    Serial.println(rpm2);
  }
}

