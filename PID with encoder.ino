/*
  Created by:

  Zahran Mhaskar aka EmbedHead
  LMR: z-world

*/

/*
  LCD RS pin to analog pin 0
   LCD Enable pin to analog pin 1
   LCD D4 pin to analog pin 2
   LCD D5 pin to analog pin 3
   LCD D6 pin to analog pin 4
   LCD D7 pin to analog pin 5
   LCD R/W pin to ground
  LCD VO pin (pin 3) to GROUND

  Connect encoders to digital pin 2 and 3
  Connect motor PWM pin to digital pin 5
  Connect motor direction pins to digital pins 6 and 7
*/

#include <LiquidCrystal.h>
//initializing all the variables
#define InA1            6
#define InB1            7
#define PWM1            5
#define encodPinA1      2
#define encodPinB1      3
#define LOOPTIME        100
#define NUMREADINGS     10


LiquidCrystal lcd(A0, A1, A2, A3, A4, A5);
int readings[NUMREADINGS];
unsigned long lastMilli = 0;
unsigned long lastMilliPrint = 0;
int speed_req = 80;                            // Set Point)
int speed_act = 0;                              //actual value
int PWM_val = 0;
volatile long count = 0; // revolution counter
volatile int count1 = 0; // revolution counter for position
float Kp = 0.4;          //setting Kp
float Kd = 1;            //setting Kd

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115600);       //establishimg serial communication at 115600 baud
  //initializing the lcd
  //setting motor pins as output
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  //setting encoder pins as input
  pinMode(encodPinA1, INPUT);
  pinMode(encodPinB1, INPUT);
  //internal pullup for encoders
  digitalWrite(encodPinA1, HIGH);
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(1, rencoder, FALLING);  //set pin no 2 as interrupt pin (INTERRUPT 1)
  for (int i = 0; i < NUMREADINGS; i++)   readings[i] = 0;
  analogWrite(PWM1, PWM_val);
  //setting motor direction as FORWARD
  digitalWrite(InA1, HIGH);
  digitalWrite(InB1, LOW);
}


///////////////**************************************************////////////////////
void loop() {
  getParam();                                                                 // check keyboard
  if ((millis() - lastMilli) >= LOOPTIME)
  { // enter timed loop
    lastMilli = millis();
    getMotorData();                                                          // calculate speed
    PWM_val = updatePid(PWM_val, speed_req, speed_act);        // compute PWM value
    analogWrite(PWM1, PWM_val);                                               // send PWM to motor
  }
  printMotorInfo();                                                           // display data
}

///////////////****************************************************//////////////////
void getMotorData()  {                                       // calculate speed
  static long countAnt = 0;                                                    // last count
  //Calculating the speed using encoder count
  speed_act = ((count - countAnt) * (60 * (1000 / LOOPTIME))) / (30);
  countAnt = count;                                           //setting count value to last count
}

int updatePid(int command, int targetValue, int currentValue)   {      // compute PWM value
  float pidTerm = 0;                                                           // PID correction
  int error = 0;
  static int last_error = 0;
  error = abs(targetValue) - abs(currentValue);
  pidTerm = (Kp * error) + (Kd * (error - last_error));
  last_error = error;
  return constrain(command + int(pidTerm), 0, 255);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void printMotorInfo()  {                                                     // display data
  if ((millis() - lastMilliPrint) >= 150)
  {
    lastMilliPrint = millis();
    int pwm_value = PWM_val;
    pwm_value = map(pwm_value, 0, 255, 0, 100);

    Serial.print("SP:");                Serial.print(speed_req);
    Serial.print("  RPM:");           Serial.print(speed_act);
    Serial.print("  PWM:");          Serial.print(PWM_val);
    Serial.print("  enc_count:");   Serial.print(count);
    Serial.print("  kp:");              Serial.print(Kp);
    Serial.print("  kd:");              Serial.println(Kd);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void rencoder()  {                                    //Read Encoder
  if (digitalRead(encodPinB1) == HIGH)
    count--;                // if encoder pin 2 = HIGH then count --
  else
    count++;                // if encoder pin 2 = LOW then count ++
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

int getParam()
{
  char param, cmd;
  if (!Serial.available())    return 0;
  delay(10);
  param = Serial.read();                              // get parameter byte
  if (!Serial.available())    return 0;
  cmd = Serial.read();                                // get command byte
  Serial.flush();                                     // clean serial buffer

  switch (param)
  {
    case 's':                                         // adjust speed more
      if (cmd == '+')
      {
        speed_req += 100;
        if (speed_req > 400)   speed_req = 400;
      }
      if (cmd == '-')
      {
        speed_req -= 100;
        if (speed_req < 0)   speed_req = 0;
      }
      break;


    /////////////////////////////////////////////
    case 'r':                                         // adjust speed slowly
      if (cmd == '+')
      {
        speed_req += 20;
        if (speed_req > 400)   speed_req = 400;
      }
      if (cmd == '-')
      {
        speed_req -= 20;
        if (speed_req < 0)   speed_req = 0;
      }
      break;
    ////////////////////////////////////////////
    case 'a':                                        // adjust direction
      if (cmd == '+')
      {
        digitalWrite(InA1, LOW);
        digitalWrite(InB1, HIGH);
      }
      if (cmd == '-')
      {
        digitalWrite(InA1, HIGH);
        digitalWrite(InB1, LOW);
      }
      break;
    ////////////////////////////////////////////
    case 'o':                                        // type "oo" to stop motor
      digitalWrite(InA1, LOW);
      digitalWrite(InB1, LOW);
      speed_req = 0;
      break;
    ////////////////////////////////////////////
    case 'p':                                       //adjust proportional gain
      if (cmd == '+')
      {
        Kp += 0.1;
      }
      if (cmd == '-')   {
        Kp -= 0.1;
      }
      break;
    ///////////////////////////////////////////
    case 'd':                                     //adjust derivative gain
      if (cmd == '+')
      {
        Kd += 0.1;
      }
      if (cmd == '-')
      {
        Kd -= 0.1;
      }
      break;
    ///////////////////////////////////////////
    default:
      Serial.println("ERROR");
  }
}
/////////////////////////////////////////////////THE END//////////////////////////////////////////////


