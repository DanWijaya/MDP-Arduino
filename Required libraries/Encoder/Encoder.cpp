#include "Arduino.h"
#include "Encoder.h"


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
}

double Encoder::getMotor1RPM(){
	static long countAnt1 = 0;
	int speed = (count1 - countAnt1) * (140.0 / 195.0);
	countAnt1 = count1;
	return speed;
}

double Encoder::getMotor2RPM(){
	static long countAnt2 = 0;
	int speed = -(count2 - countAnt2) * (146.0 / 197.0);
	countAnt2 = count2;
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

int Encoder::getCount1(){
	return count1;
}

int Encoder::getCount2(){
	return count2;
}