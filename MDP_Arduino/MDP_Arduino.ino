#include "DualVNH5019MotorShield.h"
#include <Encoder.h>
DualVNH5019MotorShield md;
//
//#define encoderPinA1    9
//#define encoderPinB1    10
//
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
//
Encoder en(3,5,11,13);
unsigned long lastMilliPrint = 0;
int count = 0;
void setup()
{
  Serial.begin(115200);
//  Serial.println("Dual VNH5019 Motor Shield");
  attachInterrupt(digitalPinToInterrupt(3), readEncoder1, FALLING);
//  md.init();
}
//

void readEncoder1(){
  en.rencoder();
}
void loop()
{
  if ((millis() - lastMilliPrint) >= 150)
  {
    lastMilliPrint = millis();
    int val = en.getRPM();
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
//
//
//////Read Encoder
////void rencoder(){
////  if (digitalRead(encodPinB1) == HIGH)
////    count--;                // if encoder pin 2 = HIGH then count --
////  else
////    count++;                // if encoder pin 2 = LOW then count ++
////}
