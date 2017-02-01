#include "Arduino.h"
#include "Encoder.h"

#define time 100

Encoder::Encoder(unsigned char pinA, unsigned char pinB, unsigned char pinC, unsigned char pinD){
	pin1 = pinA;
	pin2 = pinB;
	pin3 = pinC;
	pin4 = pinD;
	count = 0;
	speed = 0;
}

double Encoder::getSpeed(){
	return 0;
	
}

double Encoder::getRPM(){
	static long countAnt = 0;
	speed = ((count - countAnt) * (60 * (1000 / time))) / (30);
	countAnt = count;
	return speed;
}

void Encoder::rencoder()  {
  if (digitalRead(pin2) == HIGH)
    count++; 
  else
    count--;
}
