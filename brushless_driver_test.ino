
#include "Brushless.h"

#define PERIOD 10

Brushless bldc;

void setup() {
  bldc.setupTimer();
  pinMode(PC13, OUTPUT);
  Serial.begin(115200);
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
  unsigned long time_now = 0;
  loops++;
  if(millis() >= time_now + PERIOD){
    time_now += PERIOD;

    float static duty = 0;
    float tp = 0.05;
    duty = duty*(1-tp) + tp * avg_adc(PA0, 40)/40.96;
    uint16_t comp_value = bldc.setDutycyle(duty);

    printDash("Dutycycle", duty);
    printDash("Comp_value", comp_value);
    loops = 0;

    Serial.println();
  }
}
