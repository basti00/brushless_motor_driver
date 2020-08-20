/*
 * Drives the motor in open loop. The SVM rotates a vector at a constant rate.
 * ADC input varies the speed from 0 to 1000 rad/sec.
 */

#include "Brushless.h"

#define PERIOD 1
#define ADC_PIN PA0

Brushless bldc;

void setup() {
  bldc.setupPWMTimer();
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

/*
 * is true only after set amount of ms, otherwise false
 */
bool enoughTimeHasPassed(){
  unsigned long static time_now = millis();
  if(millis() >= time_now + PERIOD){
    time_now += PERIOD;
    return true;
  }
  return false;
}

void loop()
{
  if(enoughTimeHasPassed()){
    float input = getFilteredInput();
    if(input < 0.03)
      bldc.setMagnitude(0);
    else
      bldc.setMagnitude(0.8);
    bldc.setSpeed(input*input*1000);

    Serial.print(bldc.getInfo());
    Serial.println();
  }
}
