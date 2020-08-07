
#include "Brushless.h"

#define PERIOD 10

Brushless bldc;

void setup() {
  bldc.setupPWMTimer(10000);
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


void loop()
{
  int static loops = 0;
  unsigned long static time_now = 0;
  loops++;

  if(millis() >= time_now + PERIOD){
    time_now += PERIOD;

/*    bldc.setPosition(1,0);
    Serial.print(bldc.getInfo());
    Serial.println();
    return;*/

    float static duty = 0;
    float tp = 0.05;
    duty = duty*(1-tp) + tp * avg_adc(PA0, 40)/4096;
    float static angle = 0;
    angle += 0.01;
    bldc.setPosition(duty,angle);
    //bldc.setPosition(1,duty*2*PI);

    //printDash("millis", (int)millis());
    printDash("duty", duty*360);
    //printDash("Comp_value", comp_value);
    Serial.print(bldc.getInfo());
    loops = 0;

    Serial.println();
  }
}
