#include "SharpIR.h"

#include "Arduino.h"

#define LONG 20150
#define SHORT 1080

#define irFM A0
#define irFL A1
#define irFR A2
#define irLF A3
#define irLM A4
#define irRM A5

SharpIR sensorFR(irFR, 1, 93, SHORT);
SharpIR sensorFL(irFL, 1, 93, SHORT);
SharpIR sensorFM(irFM, 1, 93, SHORT);
SharpIR sensorLM(irLM, 1, 93, SHORT);
SharpIR sensorLF(irLF, 1, 93, SHORT);
SharpIR sensorRM(irRM, 1, 93, LONG);

void setup(){
  Serial.begin(9600);

  pinMode(irFR, INPUT);
  pinMode(irFL, INPUT);
  pinMode(irFM, INPUT);
  pinMode(irRM, INPUT);
  pinMode(irLF, INPUT);
  pinMode(irLM, INPUT);

  digitalWrite(irFR, LOW);
  digitalWrite(irFL, LOW);
  digitalWrite(irFM, LOW);
  digitalWrite(irRM, LOW);
  digitalWrite(irLM, LOW);
  digitalWrite(irLF, LOW);

  
}
void loop(){
  Export_Sensors();
  delay (1000);
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
      dis = sensorFM.distance();
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

