//
// Created by uhlse on 04.08.2020.
//

#ifndef ARDUINO_BRUSHLESS_H
#define ARDUINO_BRUSHLESS_H

#include "Arduino.h"

//pins to enable/disable the three half bridges
#define ENA PB8
#define ENB PB6
#define ENC PB4
//pins to switch the three half bridges
#define INA PB7
#define INB PB5
#define INC PB3
//pin numbers for direct port manipulation
#define EN_A_ 8
#define EN_B_ 6
#define EN_C_ 4
#define IN_A_ 7
#define IN_B_ 5
#define IN_C_ 3

//hard ware interrupt pins
#define HAL1 PA8
#define HAL3 PB13
#define HAL2 PA9

#define ENABLE_MASK ((1<<EN_A_)|(1<<EN_B_)|(1<<EN_C_))
#define PHASE_MASK ((1<<IN_A_)|(1<<IN_B_)|(1<<IN_C_))

typedef enum SVM_phase{
  SVM_A,
  SVM_B,
  SVM_C,
  SVM_NONE,
}SVM_phase;

const uint16_t SVM_phase_pin[] = {
        (1<<IN_A_), //SVM_A,
        (1<<IN_B_), //SVM_B,
        (1<<IN_C_),  //SVM_C,
        0  //SVM_NONE,
};

typedef enum SVM_vector{
  SVM_V0, // all connected to GND
  SVM_V1,
  SVM_V3,
  SVM_V2,
  SVM_V6,
  SVM_V4,
  SVM_V5,
  SVM_V7, // all connected to VCC
}SVM_vector;

typedef enum CNTRL_mode{
  CNTRL_fixedAngleFromHall,
  CNTRL_continousRotation,
  CNTRL_staticVector,
}CNTRL_mode;


String dash(String name, uint32_t val);
String dash(String name, uint16_t val);
String dash(String name, uint8_t val);
String dash(String name, int32_t val);
String dash(String name, int16_t val);
String dash(String name, int8_t val);
String dash(String name, int val);
String dash(String name, float val);
String dash(String name, String val);

float fMod(float a, float b);

class Brushless
{
  public:
    Brushless();
    void setupPWMTimer();

    /* void setMagnitude()
     * float m: set the magnitude(~dutycycle) of the SVM-module, in range 0 to 1.
     * can be used with all control modes.
     */
    static void setMagnitude(float mag);

    /* void setVector()
     * set the SVM module to this static vector.
     * float mag: magnitude of vector, in range 0 to 1
     * float angle: angle of vector, one rotation is 0 to 2*PI
     */
    void setVector(float mag, float angle);

    /* void setSpeed()
     * let the SVM-vector rotate at a constant rate. Does not compensate for
     * varying loads on the motor. Open loop
     */
    void setSpeed(float rad_per_sec);

    /* void setFixedAngleFromHall()
     * the SVM-module is always set to a specific angle
     * in front of the detected rotor position.
     * Closed loop, but obviously not great way of driving a motor.
     * Control speed via the magnitude.
     */
    void setFixedAngleFromHall();

    String getInfo();

  private:
    static uint16_t setAngle(float angle);
    inline int hz_to_us(int f) { return 1000000/f; }

    static float getHallAngle();
    static uint8_t resolveSektor();
    static void proc_hall();
    static void handler_hall1();
    static void handler_hall2();
    static void handler_hall3();

    static void handler_pwm1();
    static void handler_pwm2();
    static void handler_pwm3();
    static void handler_overflow();


    /*
    * to execute periodic control procedures
    * 1000Hz = 1ms period
    */
    static void handler_control();

    /*
    * hall_timer is used to time hall sensor predictions
    */
    static void handler_hall_timer();


  private:

    // essential for PWM module
    volatile static SVM_phase OCR1_phase; // needed by the ISRs, shared ressources
    volatile static SVM_phase OCR2_phase;
    volatile static SVM_phase OCR3_phase;

    // for control routine
    volatile static uint16_t control_freq_;
    volatile static CNTRL_mode cntrl_mode_;
    volatile static float radians_per_sec_;
    static float m_;     // magnitude value 0 .. 1


    // for hall sensor detection
    static volatile uint8_t hall1;
    static volatile uint8_t hall2;
    static volatile uint8_t hall3;
    static volatile uint16_t hall_int_num;
    static volatile uint8_t hall_sektor;
    static volatile uint16_t hall_rotations;
    static volatile uint8_t hall_old_sektor;
    static volatile int8_t hall_direction;
    static volatile uint8_t hall_timer_ovrflws;
    static volatile uint32_t hall_timer_predicted_ticks;

    // for debug
    static float phi_;   // angle of pwm vector in rad 0 .. 2pi
    static float alpha_; // angle of pwm vector in sektor in rad
    static float t_;     // pwm period in us
    static float t0_;    // in us, time to spent on V0 or V7
    static float t1_;    // in us, time to spent on vx
    static float t2_;    // in us, time to spent on vy
    static uint16_t OCR1_; // timer overflow values
    static uint16_t OCR2_;
    static uint16_t OCR3_;
    volatile static SVM_vector vx_; // redundant! the two vectors to modulate with
    volatile static SVM_vector vy_;
};


#endif //ARDUINO_BRUSHLESS_H
