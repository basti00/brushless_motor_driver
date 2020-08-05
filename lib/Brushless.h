//
// Created by uhlse on 04.08.2020.
//

#ifndef ARDUINO_BRUSHLESS_H
#define ARDUINO_BRUSHLESS_H

#include "Arduino.h"

//pin definition
#define EN1 PB8
#define EN2 PB6
#define EN3 PB4
#define IN1 PB7
#define IN2 PB5
#define IN3 PB3
#define IN1_ 7
#define IN2_ 5
#define IN3_ 3

#define HAL1 PA8
#define HAL2 PB13
#define HAL3 PA9

void handler_pwm_low();
void handler_pwm_high();

String dash(String name, int val);
String dash(String name, float val);
String dash(String name, String val);

class Brushless
{
  public:
    Brushless();
    void setupTimer();
    uint16_t setDutycyle(float dutycycle);
    String getInfo();
    inline int hz_to_us(int f) { return 1000000/f; }

    static void calc_rotation();
    static void Hall1_ISR();
    static void Hall2_ISR();
    static void Hall3_ISR();
    static void handler_sections();

    static void handler_pwm_low();
    static void handler_pwm_high();


  private:
    static volatile uint8_t hall1;
    static volatile uint8_t hall2;
    static volatile uint8_t hall3;
    static volatile uint16_t hall_int_num;
    static volatile uint8_t hall_sektor;
    static volatile uint16_t hall_rotations;
    static volatile uint8_t hall_old_sektor;


};


#endif //ARDUINO_BRUSHLESS_H
