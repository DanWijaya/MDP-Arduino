#include "Encoder.h"
#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"


#define enA1    3
#define enB1    5
#define enA2    11
#define enB2    13

DualVNH5019MotorShield md;
Encoder en(enA1,enB1,enA2,enB2);
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

void readEncoder1(){
  en.rencoder1();
}

void readEncoder2(){
  en.rencoder2();
}

void loop()
{
  if ((millis() - lastMilliPrint) >= 150)
  {
    lastMilliPrint = millis();
    int val = en.getMotor1RPM();
    if (val<2000){
      Serial.println(val);
    }
  }
//  for (int i = 0; i <= 400; i++)
//  {
//    md.setM1Speed(i);
//    stopIfFault();
//    if (i%200 == 100)
//    {
//      Serial.print("M1 current: ");
//      Serial.println(md.getM1CurrentMilliamps());
//    }
//    delay(2);
//  }
//  
//  for (int i = 400; i >= -400; i--)
//  {
//    md.setM1Speed(i);
//    stopIfFault();
//    if (i%200 == 100)
//    {
//      Serial.print("M1 current: ");
//      Serial.println(md.getM1CurrentMilliamps());
//    }
//    delay(2);
//  }
//  
//  for (int i = -400; i <= 0; i++)
//  {
//    md.setM1Speed(i);
//    stopIfFault();
//    if (i%200 == 100)
//    {
//      Serial.print("M1 current: ");
//      Serial.println(md.getM1CurrentMilliamps());
//    }
//    delay(2);
//  }
//
//  for (int i = 0; i <= 400; i++)
//  {
//    md.setM2Speed(i);
//    stopIfFault();
//    if (i%200 == 100)
//    {
//      Serial.print("M2 current: ");
//      Serial.println(md.getM2CurrentMilliamps());
//    }
//    delay(2);
//  }
//  
//  for (int i = 400; i >= -400; i--)
//  {
//    md.setM2Speed(i);
//    stopIfFault();
//    if (i%200 == 100)
//    {
//      Serial.print("M2 current: ");
//      Serial.println(md.getM2CurrentMilliamps());
//    }
//    delay(2);
//  }
//  
//  for (int i = -400; i <= 0; i++)
//  {
//    md.setM2Speed(i);
//    stopIfFault();
//    if (i%200 == 100)
//    {
//      Serial.print("M2 current: ");
//      Serial.println(md.getM2CurrentMilliamps());
//    }
//    delay(2);
//  }
}
