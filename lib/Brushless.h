//
// Created by uhlse on 04.08.2020.
//

#ifndef ARDUINO_BRUSHLESS_H
#define ARDUINO_BRUSHLESS_H

#include "Arduino.h"

//pin definition
#define ENA PB8
#define ENB PB6
#define ENC PB4
#define INA PB7
#define INB PB5
#define INC PB3
#define EN_A_ 8
#define EN_B_ 6
#define EN_C_ 4
#define IN_A_ 7
#define IN_B_ 5
#define IN_C_ 3

#define HAL1 PA8
#define HAL2 PB13
#define HAL3 PA9

#define ENABLE_ALL ((1<<EN_A_)|(1<<EN_B_)|(1<<EN_C_))

typedef enum SVM_vector{
  SVM_V0, // connected to GND
  SVM_V1,
  SVM_V3,
  SVM_V2,
  SVM_V6,
  SVM_V4,
  SVM_V5,
  SVM_V7, //connected to VCC
  SVM_ENA  //enable all
}SVM_vector;

// using hardcoded pin-configs (todo make dynamic)
const uint16_t SVM_HW_pins[] = {
        0,                       //SVM_V0, // connected to GND
        (1<<IN_A_),              //SVM_V1,
        (1<<IN_A_) | (1<<IN_B_), //SVM_V3,
        (1<<IN_B_),              //SVM_V2,
        (1<<IN_B_) | (1<<IN_C_), //SVM_V6,
        (1<<IN_C_),              //SVM_V4,
        (1<<IN_C_) | (1<<IN_A_), //SVM_V5,
        (1<<IN_A_) | (1<<IN_B_) | (1<<IN_C_), //SVM_V7, //connected to VCC
        (1<<EN_A_) | (1<<EN_B_) | (1<<EN_C_), //SVM_ENA
};


void handler_pwm_low();
void handler_pwm_high();

String dash(String name, int val);
String dash(String name, float val);
String dash(String name, String val);
float fMod(float a, float b);

class Brushless
{
  public:
    Brushless();
    void setupPWMTimer(uint32_t f);
    uint16_t setDutycyle(float dutycycle);
    uint16_t setPosition(float m, float angle);
    String getInfo();
    inline int hz_to_us(int f) { return 1000000/f; }

    static void calc_rotation();
    static void Hall1_ISR();
    static void Hall2_ISR();
    static void Hall3_ISR();
    static void handler_sections();

    static void handler_pwm1();
    static void handler_pwm2();
    static void handler_pwm3();
    static void handler_overflow();
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

    float m_;     // 0 - 1
    float alpha_; // angle in sektor in rad
    float t_;  // pwm period in us
    float t0_; // in us, time to spent on V0 or V7
    float t1_; // in us, time to spent on vx
    float t2_; // in us, time to spent on vy
    volatile static SVM_vector vx_; // the two vectors to modulate with
    volatile static SVM_vector vy_;


};


#endif //ARDUINO_BRUSHLESS_H
