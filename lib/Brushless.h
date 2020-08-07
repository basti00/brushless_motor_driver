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

#define ENABLE_MASK ((1<<EN_A_)|(1<<EN_B_)|(1<<EN_C_))
#define PHASE_MASK ((1<<IN_A_)|(1<<IN_B_)|(1<<IN_C_))

typedef enum SVM_phase{
  SVM_A,
  SVM_B,
  SVM_C,
  SVM_NONE,
}SVM_phase;

const uint8_t SVM_phase_pin[] = {
        (1<<IN_A_), //SVM_A,
        (1<<IN_B_), //SVM_B,
        (1<<IN_C_),  //SVM_C,
        0  //SVM_NONE,
};

typedef enum SVM_vector{
  SVM_V0, // connected to GND
  SVM_V1,
  SVM_V3,
  SVM_V2,
  SVM_V6,
  SVM_V4,
  SVM_V5,
  SVM_V7, //connected to VCC
}SVM_vector;

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

    // essential
    volatile static SVM_phase OCR1_phase; // needed by the ISRs, shared ressources todo locking
    volatile static SVM_phase OCR2_phase;
    volatile static SVM_phase OCR3_phase;

    // for debug
    float m_;     // 0 - 1
    float alpha_; // angle in sektor in rad
    float t_;  // pwm period in us
    float t0_; // in us, time to spent on V0 or V7
    float t1_; // in us, time to spent on vx
    float t2_; // in us, time to spent on vy
    uint16_t OCR1_; // timer overflow values
    uint16_t OCR2_;
    uint16_t OCR3_;
    volatile static SVM_vector vx_; // redundant! the two vectors to modulate with
    volatile static SVM_vector vy_;
};


#endif //ARDUINO_BRUSHLESS_H
