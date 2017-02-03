#include "Arduino.h"
#include "Encoder.h"

#define time 100

Encoder::Encoder(unsigned char pinA, unsigned char pinB, unsigned char pinC, unsigned char pinD){
	pinA1 = pinA;
	pinB1 = pinB;
	pinA2 = pinC;
	pinB2 = pinD;
}

void Encoder::init(){
	pinMode(pinA1, INPUT);
	pinMode(pinB1, INPUT);
	pinMode(pinA2, INPUT);
	pinMode(pinB2, INPUT);
	count1 = 0;
	count2 = 0;
	speed = 0;
}

double Encoder::getMotor1RPM(){
	static long countAnt1 = 0;
	speed = ((count - countAnt) * (60 * (1000 / time))) / (30);
	countAnt1 = count;
	return speed;
}

double Encoder::getMotor2RPM(){
	static long countAnt2 = 0;
	speed = -((count - countAnt) * (60 * (1000 / time))) / (30);
	countAnt2 = count;
	return speed;
}

void Encoder::rencoder1()  {
  if (digitalRead(pinB1) == HIGH)
    count1++; 
  else
    count1--;
}

void Encoder::rencoder2()  {
  if (digitalRead(pinB2) == HIGH)
    count2++; 
  else
    count2--;
}
