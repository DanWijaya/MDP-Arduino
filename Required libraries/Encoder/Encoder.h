#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"

class Encoder
{
	private:
		unsigned char pin1;
		unsigned char pin2;
		unsigned char pin3;
		unsigned char pin4;
		volatile unsigned int count;
		double speed;
		
	public:
		//constructor
		Encoder(unsigned char pinA, unsigned char pinB, unsigned char pinC, unsigned char pinD);
	
		void rencoder();
		double getSpeed();
		double getRPM();
};

#endif