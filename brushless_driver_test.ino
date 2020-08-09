
#include "Brushless.h"

#define PERIOD 10

Brushless bldc;

void setup() {
  bldc.setupPWMTimer();
  pinMode(PC13, OUTPUT);
  Serial.begin(115200);
  delay(1);

}

void printDash(String name, int val){
  Serial.print(dash(name,val));
}
void printDash(String name, float val){
  Serial.print(dash(name,val));
}
void printDash(String name, String val){
  Serial.print(dash(name,val));
}

int avg_adc(int pin, int n){
  long sum = 0;
  for(int i = 0; i<n; i++){
    sum += analogRead(pin);
  }
  return sum/n;
}

float deg = 6*PI/180;

void loop()
{
  int static loops = 0;
  unsigned long static time_now = millis();
  loops++;

  if(millis() >= time_now + PERIOD){
    time_now += PERIOD;

    float static poti = 0;
    float tp = 0.05;
    poti = poti*(1-tp) + tp * avg_adc(PA0, 40)/4096; // 0..1
    //float static angle = 0;
    //angle += (6*PI/180) * duty;

    bldc.setRPS(poti*poti * 100);
    //bldc.setPosition(0.8,angle);
    //bldc.setPosition(duty,angle);
    //bldc.setPosition(0.8,duty*3*PI); // one and a half rotation

    //printDash("millis", (int)millis());
    printDash("poti", poti);
    //printDash("Comp_value", comp_value);
    //Serial.print(bldc.getInfo());

    Serial.println();
  }
}
