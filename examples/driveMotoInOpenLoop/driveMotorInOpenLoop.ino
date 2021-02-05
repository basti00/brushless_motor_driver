#include "Brushless.h"

#define PERIOD 1
#define ADC_PIN PA0

Brushless bldc;

void setup() {
  bldc.setupPWMTimer();
  pinMode(PC13, OUTPUT);
  Serial.begin(115200);
  delay(1);
}

/*
 * returns average of n samples
 */
int avg_adc(int pin, int n){
  long sum = 0;
  for(int i = 0; i<n; i++){
    sum += analogRead(pin);
  }
  return sum/n;
}

/*
 * get lowpass filtered input, scaled to 0 .. 1
 */
float getFilteredInput(){
  float static inp = 0;
  float tp = 0.05;
  inp = inp * (1-tp) + tp * avg_adc(ADC_PIN, 40)/4096;
}

float deg = 6*PI/180;

void loop()
{
  int static loops = 0;
  unsigned long static time_now = millis();
  loops++;

  if(millis() >= time_now + PERIOD){
    time_now += PERIOD;

    float input = getFilteredInput();
    if(input < 0.03)
      bldc.setMagnitude(0);
    else
      bldc.setMagnitude(input);
    bldc.setFixedAngleFromHall();


    Serial.print(bldc.getInfo());
    //printDash("free_loops", loops);
    loops = 0;
    Serial.println();
  }
}
